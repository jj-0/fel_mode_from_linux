/*
 * fel_mode.c - Enter FEL mode on Allwinner SoCs from Linux
 *
 * Writes the FEL magic value to the appropriate register and triggers
 * a watchdog reset to enter FEL mode.
 * Must be run as root.
 *
 * Supported SoCs:
 *   A20        (sun7i,  0x1651)  - confirmed working
 *   A33/R16    (sun8i,  0x1667)  - FEL addr from BROM source (bsp_for_a33)
 *   H3/H2+    (sun8i,  0x1680)  - FEL addr from vendor u-boot (sun8iw7)
 *   H5        (sun50i, 0x1718)  - same R_PRCM layout as H3
 *   H6        (sun50i, 0x1728)  - RTC base at 0x07000000
 *
 * Usage: fel_mode [-r|--reboot]
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

/*
 * SoC identification register
 * Set bit 15 to enable version readout, SoC ID is in upper 16 bits
 */
#define SUNXI_SRAM_BASE      0x01C00000
#define SUNXI_SRAM_VER_REG   0x24

/*
 * SoC IDs (from version register >> 16)
 */
#define SOC_ID_A20           0x1651
#define SOC_ID_A33           0x1667  /* also R16, Zuiki z7213 */
#define SOC_ID_H3            0x1680  /* also H2+ */
#define SOC_ID_H5            0x1718
#define SOC_ID_H6            0x1728

/*
 * FEL magic value - same for all supported SoCs
 */
#define FEL_MAGIC            0x5AA5A55A

/*
 * FEL flag register addresses:
 *
 * A20:     Timer GP Register 1 at 0x01C20C00 + 0x124 = 0x01C20D24
 *          (from BROM source bsp_for_a20/common/common.c)
 *
 * A33/R16: R_PRCM RTC GP Register at 0x01F00000 + 0x108 = 0x01F00108
 *          (from BROM source bsp_for_a33/common/common.c)
 *
 * H3:      RTC GP Register 2 at 0x01F00000 + 0x100 + 2*4 = 0x01F00108
 *          (from vendor u-boot arch-sun8iw7/cpu.h)
 *
 * H5:      Same R_PRCM layout as H3 = 0x01F00108
 *
 * H6:      RTC moved to 0x07000000, GP Register 2 = 0x07000108
 */
#define FEL_ADDR_A20         0x01C20D24
#define FEL_ADDR_R_PRCM      0x01F00108  /* A33, H3, H5 */
#define FEL_ADDR_H6          0x07000108

/*
 * Watchdog registers:
 *
 * A20:      0x01C20C90 (cfg at +0x00, mode at +0x04)
 *           cfg=1 (system reset), mode=3 (enable, 0.5s)
 *
 * A33/H3/H5: 0x01C20CB8 (mode register only)
 *            mode=1 (enable reset)
 *
 * H6:      0x030090B8 (mode register only)
 *          mode=1 (enable reset)
 */
#define WDT_A20_BASE         0x01C20C90
#define WDT_H3_MODE          0x01C20CB8
#define WDT_H6_MODE          0x030090B8

/*
 * Watchdog types
 */
enum wdt_type {
    WDT_TYPE_A20,   /* Separate cfg + mode registers */
    WDT_TYPE_H3,    /* Single mode register, value 1 */
};

/*
 * SoC descriptor
 */
struct soc_info {
    const char *name;
    uint16_t id;
    uint32_t fel_flag_reg;
    uint32_t wdt_base;
    enum wdt_type wdt_type;
};

static const struct soc_info soc_table[] = {
    {
        .name = "A20 (sun7i)",
        .id = SOC_ID_A20,
        .fel_flag_reg = FEL_ADDR_A20,
        .wdt_base = WDT_A20_BASE,
        .wdt_type = WDT_TYPE_A20,
    },
    {
        .name = "A33/R16 (sun8i)",
        .id = SOC_ID_A33,
        .fel_flag_reg = FEL_ADDR_R_PRCM,
        .wdt_base = WDT_H3_MODE,
        .wdt_type = WDT_TYPE_H3,
    },
    {
        .name = "H3/H2+ (sun8i)",
        .id = SOC_ID_H3,
        .fel_flag_reg = FEL_ADDR_R_PRCM,
        .wdt_base = WDT_H3_MODE,
        .wdt_type = WDT_TYPE_H3,
    },
    {
        .name = "H5 (sun50i)",
        .id = SOC_ID_H5,
        .fel_flag_reg = FEL_ADDR_R_PRCM,
        .wdt_base = WDT_H3_MODE,
        .wdt_type = WDT_TYPE_H3,
    },
    {
        .name = "H6 (sun50i)",
        .id = SOC_ID_H6,
        .fel_flag_reg = FEL_ADDR_H6,
        .wdt_base = WDT_H6_MODE,
        .wdt_type = WDT_TYPE_H3,
    },
    { .name = NULL } /* sentinel */
};

/* Memory barrier macros for ARM */
#if defined(__arm__) || defined(__aarch64__)
#define dsb()  __asm__ __volatile__ ("dsb" : : : "memory")
#define dmb()  __asm__ __volatile__ ("dmb" : : : "memory")
#else
#define dsb()  __sync_synchronize()
#define dmb()  __sync_synchronize()
#endif

static inline void writel(uint32_t value, volatile void *addr)
{
    dmb();
    *(volatile uint32_t *)addr = value;
    dsb();
}

static inline uint32_t readl(volatile void *addr)
{
    uint32_t value;
    value = *(volatile uint32_t *)addr;
    dmb();
    return value;
}

static void *map_physical(int fd, off_t phys_addr, size_t size, off_t *offset_out)
{
    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size < 0)
        page_size = 4096;

    off_t page_addr = phys_addr & ~(page_size - 1);
    *offset_out = phys_addr - page_addr;

    void *map = mmap(NULL, size + *offset_out, PROT_READ | PROT_WRITE,
                     MAP_SHARED, fd, page_addr);
    return map;
}

static const struct soc_info *detect_soc(int fd)
{
    void *sram_base;
    off_t offset;
    uint32_t ver_reg, soc_id;
    const struct soc_info *soc;

    sram_base = map_physical(fd, SUNXI_SRAM_BASE, 0x100, &offset);
    if (sram_base == MAP_FAILED) {
        fprintf(stderr, "Failed to map SRAM for SoC detection: %s\n", strerror(errno));
        return NULL;
    }

    volatile uint32_t *ver_ptr = (volatile uint32_t *)((char *)sram_base + offset + SUNXI_SRAM_VER_REG);

    /* Save original value */
    ver_reg = readl(ver_ptr);

    /* Set bit 15 to enable SoC version readout */
    writel(ver_reg | 0x8000, ver_ptr);

    /* Read SoC ID from upper 16 bits */
    soc_id = readl(ver_ptr) >> 16;

    /* Restore original value */
    writel(ver_reg, ver_ptr);

    munmap(sram_base, 0x100 + offset);

    printf("Detected SoC ID: 0x%04X\n", soc_id);

    for (soc = soc_table; soc->name; soc++) {
        if (soc->id == soc_id)
            return soc;
    }

    fprintf(stderr, "Unsupported SoC ID: 0x%04X\n", soc_id);
    fprintf(stderr, "Supported SoCs:");
    for (soc = soc_table; soc->name; soc++)
        fprintf(stderr, " %s (0x%04X)", soc->name, soc->id);
    fprintf(stderr, "\n");
    return NULL;
}

static void trigger_watchdog_reset(int fd, const struct soc_info *soc)
{
    void *wdt_base;
    off_t offset;

    wdt_base = map_physical(fd, soc->wdt_base, 0x10, &offset);
    if (wdt_base == MAP_FAILED) {
        fprintf(stderr, "Failed to map watchdog registers: %s\n", strerror(errno));
        return;
    }

    printf("Triggering watchdog reset...\n");

    switch (soc->wdt_type) {
    case WDT_TYPE_A20:
        /* A20: separate cfg and mode registers at base+0x00 and base+0x04 */
        writel(1, (volatile uint32_t *)((char *)wdt_base + offset + 0x00)); /* cfg: system reset */
        writel(3, (volatile uint32_t *)((char *)wdt_base + offset + 0x04)); /* mode: enable, 0.5s */
        break;
    case WDT_TYPE_H3:
        /* H3/A33/H5/H6: single mode register */
        writel(1, (volatile uint32_t *)((char *)wdt_base + offset));
        break;
    }

    munmap(wdt_base, 0x10 + offset);
}

int main(int argc, char *argv[])
{
    int do_reboot = 0;
    int fd;
    const struct soc_info *soc;
    void *fel_base;
    off_t offset;
    volatile uint32_t *fel_reg;
    uint32_t readback, initial;

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--reboot") == 0) {
            do_reboot = 1;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [-r|--reboot]\n", argv[0]);
            printf("  -r, --reboot  Trigger watchdog reset after writing FEL magic\n");
            printf("\nWrites the FEL magic value to the appropriate register and\n");
            printf("optionally triggers a watchdog reset to enter FEL mode.\n");
            printf("\nSupported SoCs:\n");
            for (const struct soc_info *s = soc_table; s->name; s++)
                printf("  %-20s (0x%04X)  FEL flag at 0x%08X\n",
                       s->name, s->id, s->fel_flag_reg);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return 1;
        }
    }

    /* Open /dev/mem */
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("Failed to open /dev/mem (are you root?)");
        return 1;
    }

    /* Detect SoC */
    soc = detect_soc(fd);
    if (!soc) {
        close(fd);
        return 1;
    }

    printf("SoC: %s\n", soc->name);
    printf("FEL flag register: 0x%08X\n", soc->fel_flag_reg);

    /* Map FEL flag register */
    fel_base = map_physical(fd, soc->fel_flag_reg, 0x10, &offset);
    if (fel_base == MAP_FAILED) {
        fprintf(stderr, "Failed to map FEL flag register: %s\n", strerror(errno));
        close(fd);
        return 1;
    }

    fel_reg = (volatile uint32_t *)((char *)fel_base + offset);

    /* Read initial value */
    initial = readl(fel_reg);
    printf("Initial value: 0x%08X\n", initial);

    /* Write the magic value */
    printf("Writing FEL magic 0x%08X...\n", FEL_MAGIC);
    writel(FEL_MAGIC, fel_reg);

    /* Small delay */
    usleep(1000);

    /* Read back and verify */
    readback = readl(fel_reg);
    printf("Read back:     0x%08X\n", readback);

    if (readback == FEL_MAGIC) {
        printf("Success! FEL magic value written correctly.\n");
    } else {
        printf("Warning: Read back value does not match!\n");
    }

    munmap(fel_base, 0x10 + offset);

    /* Trigger watchdog reset if requested */
    if (do_reboot) {
        printf("\nSync filesystems...\n");
        sync();
        sync();
        trigger_watchdog_reset(fd, soc);

        /* Wait for reset - should not return */
        printf("Waiting for watchdog reset...\n");
        sleep(5);
        fprintf(stderr, "Watchdog reset failed!\n");
        close(fd);
        return 1;
    } else {
        printf("\nReboot skipped. Run with -r to trigger watchdog reset into FEL mode.\n");
    }

    close(fd);
    return 0;
}
