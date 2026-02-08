/*
 * fel_mode.c - Enter FEL mode on Allwinner SoCs from Linux
 *
 * Writes the FEL magic value to the appropriate register and triggers
 * a watchdog reset to enter FEL mode.
 * Must be run as root.
 *
 * Supported SoCs: A20 (sun7i), H3 (sun8i)
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
 */
#define SUNXI_SRAM_BASE      0x01C00000
#define SUNXI_SRAM_VER_REG   0x24        /* Version register offset */

/*
 * SoC IDs (from version register >> 16)
 */
#define SOC_ID_A20           0x1651      /* sun7i */
#define SOC_ID_H3            0x1680      /* sun8i */

/*
 * FEL magic value
 */
#define FEL_MAGIC            0x5AA5A55A

/*
 * A20 (sun7i) memory map:
 * - Timer base:         0x01C20C00
 * - Watchdog:           0x01C20C90 (cfg at +0x00, mode at +0x04)
 * - FEL flag:           0x01C20D24 (Timer GP Register 1, offset 0x124 from timer base)
 */
#define A20_FEL_FLAG_REG     0x01C20D24
#define A20_WDT_BASE         0x01C20C90
#define A20_WDT_CFG          0x00
#define A20_WDT_MODE         0x04

/*
 * H3 (sun8i) memory map:
 * - R_PRCM base:        0x01F00000
 * - FEL flag:           0x01F00108 (offset 0x108 from R_PRCM base)
 * - Watchdog:           0x01C20CB8 (mode register, different from A20)
 */
#define H3_FEL_FLAG_REG      0x01F00108
#define H3_WDT_MODE          0x01C20CB8

/*
 * SoC descriptor
 */
struct soc_info {
    const char *name;
    uint16_t id;
    uint32_t fel_flag_reg;
    uint32_t wdt_base;
    int wdt_has_cfg;  /* A20 has separate cfg register, H3 doesn't */
};

static const struct soc_info soc_a20 = {
    .name = "A20 (sun7i)",
    .id = SOC_ID_A20,
    .fel_flag_reg = A20_FEL_FLAG_REG,
    .wdt_base = A20_WDT_BASE,
    .wdt_has_cfg = 1,
};

static const struct soc_info soc_h3 = {
    .name = "H3 (sun8i)",
    .id = SOC_ID_H3,
    .fel_flag_reg = H3_FEL_FLAG_REG,
    .wdt_base = H3_WDT_MODE,
    .wdt_has_cfg = 0,
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

    switch (soc_id) {
    case SOC_ID_A20:
        return &soc_a20;
    case SOC_ID_H3:
        return &soc_h3;
    default:
        fprintf(stderr, "Unsupported SoC ID: 0x%04X\n", soc_id);
        fprintf(stderr, "Supported SoCs: A20 (0x1651), H3 (0x1680)\n");
        return NULL;
    }
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

    if (soc->wdt_has_cfg) {
        /* A20 style: separate cfg and mode registers */
        volatile uint32_t *wdt_cfg = (volatile uint32_t *)((char *)wdt_base + offset + A20_WDT_CFG);
        volatile uint32_t *wdt_mode = (volatile uint32_t *)((char *)wdt_base + offset + A20_WDT_MODE);

        writel(1, wdt_cfg);   /* Reset whole system */
        writel(3, wdt_mode);  /* Enable watchdog, 0.5s interval */
    } else {
        /* H3 style: single mode register */
        volatile uint32_t *wdt_mode = (volatile uint32_t *)((char *)wdt_base + offset);

        writel(1, wdt_mode);  /* Enable watchdog reset */
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
            printf("\nThis tool writes the FEL magic value to the appropriate register\n");
            printf("and optionally triggers a watchdog reset to enter FEL mode.\n");
            printf("\nSupported SoCs: A20 (sun7i), H3 (sun8i)\n");
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
