/*
 *  eee.c - Asus eeePC extras
 *
 *  Copyright (C) 2007 Andrew Tipton
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Ths program is distributed in the hope that it will be useful,
 *  but WITOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTAILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Template Place, Suite 330, Boston, MA  02111-1307 USA
 *  
 *  ---------
 *
 *  WARNING:  This is an extremely *experimental* module!  This code has been
 *  developed through trial-and-error, which means I don't really understand
 *  100% what's going on here...  That means there's a chance that there could
 *  be unintended side-effects which might cause data loss or even physical
 *  damage!
 *
 *  Again, this code comes WITHOUT ANY WARRANTY whatsoever.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>             // For inb() and outb()
#include <linux/i2c.h>
#include <linux/mutex.h>


/* Module info */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrew Tipton");
MODULE_DESCRIPTION("Support for eeePC-specific functionality.");
#define EEE_VERSION "0.2"


/* PLL access functions.
 *
 * Note that this isn't really the "proper" way to use the I2C API... :)
 * I2C_SMBUS_BLOCK_MAX is 32, the maximum size of a block read/write.
 */
static void eee_pll_init(void);
static void eee_pll_read(void);
static void eee_pll_write(void);
static void eee_pll_cleanup(void);


static struct i2c_client eee_pll_smbus_client = {
    .adapter = NULL,
    .addr = 0x69,
    .flags = 0,
};
static char eee_pll_data[I2C_SMBUS_BLOCK_MAX];
static int eee_pll_datalen = 0;

static void eee_pll_init(void) {
    eee_pll_smbus_client.adapter = i2c_get_adapter(0);

    // Fill the eee_pll_data buffer.
    eee_pll_read();
}

// Takes approx 150ms to execute.
static void eee_pll_read(void) {
    memset(eee_pll_data, 0, I2C_SMBUS_BLOCK_MAX);
    eee_pll_datalen = i2c_smbus_read_block_data(&eee_pll_smbus_client, 0, eee_pll_data);
}

// Takes approx 150ms to execute ???
static void eee_pll_write(void) {
    i2c_smbus_write_block_data(&eee_pll_smbus_client, 0, eee_pll_datalen, eee_pll_data);
}

static void eee_pll_cleanup(void) {
    i2c_put_adapter(eee_pll_smbus_client.adapter);
}

/* Embedded controller access functions.
 *
 * The ENE KB3310 embedded controller has a feature known as "Index IO"
 * which allows the entire 64KB address space of the controller to be
 * accessed via a set of ISA I/O ports at 0x380-0x384.  This allows us
 * direct access to all of the controller's ROM, RAM, SFRs, and peripheral
 * registers;  this access bypasses the EC firmware entirely.
 *
 * This is much faster than using ec_transaction(), and it also allows us to
 * do things which are not possible through the EC's official interface.
 *
 * An Indexed IO write to an EC register takes approx. 90us, while an EC
 * transaction takes approx. 2500ms.
 */
#define EC_IDX_ADDRH 0x381
#define EC_IDX_ADDRL 0x382
#define EC_IDX_DATA 0x383
#define HIGH_BYTE(x) ((x & 0xff00) >> 8)
#define LOW_BYTE(x) (x & 0x00ff)
static DEFINE_MUTEX(eee_ec_mutex);

static unsigned char eee_ec_read(unsigned short addr) {
    unsigned char data;

    mutex_lock(&eee_ec_mutex);
    outb(HIGH_BYTE(addr), EC_IDX_ADDRH);
    outb(LOW_BYTE(addr), EC_IDX_ADDRL);
    data = inb(EC_IDX_DATA);
    mutex_unlock(&eee_ec_mutex);

    return data;
}

static void eee_ec_write(unsigned short addr, unsigned char data) {
    mutex_lock(&eee_ec_mutex);
    outb(HIGH_BYTE(addr), EC_IDX_ADDRH);
    outb(LOW_BYTE(addr), EC_IDX_ADDRL);
    outb(data, EC_IDX_DATA);
    mutex_unlock(&eee_ec_mutex);
}

static void eee_ec_gpio_set(int pin, int value) {
    unsigned short port;
    unsigned char mask;

    port = 0xFC20 + ((pin >> 3) & 0x1f);
    mask = 1 << (pin & 0x07);
    if (value) {
        eee_ec_write(port, eee_ec_read(port) | mask);
    } else {
        eee_ec_write(port, eee_ec_read(port) & ~mask);
    }
}

static int eee_ec_gpio_get(int pin) {
    unsigned short port;
    unsigned char mask;
    unsigned char status;

    port = 0xfc20 + ((pin >> 3) & 0x1f);
    mask = 1 << (pin & 0x07);
    status = eee_ec_read(port) & mask;

    return (status) ? 1 : 0;
}

/*** Fan and temperature functions ***/
#define EC_ST00 0xF451          // Temperature of CPU (C)
#define EC_SC02 0xF463          // Fan PWM duty cycle (%)
#define EC_SC05 0xF466          // High byte of fan speed (RPM)
#define EC_SC06 0xF467          // Low byte of fan speed (RPM)
#define EC_SFB3 0xF4D3          // Flag byte containing SF25 (FANctrl)

static unsigned int eee_get_temperature(void) {
    return eee_ec_read(EC_ST00);
}

static unsigned int eee_fan_get_rpm(void) {
    return (eee_ec_read(EC_SC05) << 8) | eee_ec_read(EC_SC06);
}

// 1 if fan is in manual mode, 0 if controlled by the EC
static int eee_fan_get_manual(void) {
    return (eee_ec_read(EC_SFB3) & 0x02) ? 1 : 0;
}

static void eee_fan_set_manual(int manual) {
    if (manual) {
        // SF25=1: Prevent the EC from controlling the fan.
        eee_ec_write(EC_SFB3, eee_ec_read(EC_SFB3) | 0x02);
    } else {
        // SF25=0: Allow the EC to control the fan.
        eee_ec_write(EC_SFB3, eee_ec_read(EC_SFB3) & ~0x02);
    }
}

static void eee_fan_set_speed(unsigned int speed) {
    eee_ec_write(EC_SC02, (speed > 100) ? 100 : speed);
}

static unsigned int eee_fan_get_speed(void) {
    return eee_ec_read(EC_SC02);
}


/*** Voltage functions ***/
#define EC_VOLTAGE_PIN 0x66
enum eee_voltage { Low=0, High=1 };
static enum eee_voltage eee_get_voltage(void) {
    return eee_ec_gpio_get(EC_VOLTAGE_PIN);
}
static void eee_set_voltage(enum eee_voltage voltage) {
    eee_ec_gpio_set(EC_VOLTAGE_PIN, voltage);
}

/*** FSB functions ***/
static void eee_get_freq(int *n, int *m) {
    *m = eee_pll_data[11] & 0x3F;
    *n = eee_pll_data[12];
}

static void eee_set_freq(int n, int m) {
    int current_n = 0, current_m = 0;
    eee_get_freq(&current_n, &current_m);
    if (current_n != n || current_m != m) {
        eee_pll_data[11] = m & 0x3F;
        eee_pll_data[12] = n & 0xFF;
        eee_pll_write();
    }
}

/*** /proc file functions ***/

static struct proc_dir_entry *eee_proc_rootdir;
#define EEE_PROC_READFUNC(NAME) \
    void eee_proc_readfunc_##NAME (char *buf, int buflen, int *bufpos)
#define EEE_PROC_WRITEFUNC(NAME) \
    void eee_proc_writefunc_##NAME (const char *buf, int buflen, int *bufpos)
#define EEE_PROC_PRINTF(FMT, ARGS...) \
    *bufpos += snprintf(buf + *bufpos, buflen - *bufpos, FMT, ##ARGS)
#define EEE_PROC_SCANF(COUNT, FMT, ARGS...) \
    do { \
        int len = 0; \
        int cnt = sscanf(buf + *bufpos, FMT "%n", ##ARGS, &len); \
        if (cnt < COUNT) { \
            printk(KERN_DEBUG "eee:  scanf(\"%s\") wanted %d args, but got %d.\n", FMT, COUNT, cnt); \
            return; \
        } \
        *bufpos += len; \
    } while (0)
#define EEE_PROC_MEMCPY(SRC, SRCLEN) \
    do { \
        int len = SRCLEN; \
        if (len > (buflen - *bufpos)) \
            len = buflen - *bufpos; \
        memcpy(buf + *bufpos, SRC, (SRCLEN > (buflen - *bufpos)) ? (buflen - *bufpos) : SRCLEN); \
        *bufpos += len; \
    } while (0)
#define EEE_PROC_FILES_BEGIN \
    static struct eee_proc_file eee_proc_files[] = {
#define EEE_PROC_RW(NAME, MODE) \
    { #NAME, MODE, &eee_proc_readfunc_##NAME, &eee_proc_writefunc_##NAME }
#define EEE_PROC_RO(NAME, MODE) \
    { #NAME, MODE, &eee_proc_readfunc_##NAME, NULL }
#define EEE_PROC_FILES_END \
    { NULL, 0, NULL, NULL } };

struct eee_proc_file {
    char *name;
    int mode;
    void (*readfunc)(char *buf, int buflen, int *bufpos);
    void (*writefunc)(const char *buf, int buflen, int *bufpos);
};


EEE_PROC_READFUNC(fsb) {
    int n = 0;
    int m = 0;
    int voltage = 0;
    eee_get_freq(&n, &m);
    voltage = (int)eee_get_voltage();
    EEE_PROC_PRINTF("%d %d %d\n", n, m, voltage);
}

EEE_PROC_WRITEFUNC(fsb) {
    int n = 70;     // sensible defaults
    int m = 24;
    int voltage = 0;
    EEE_PROC_SCANF(3, "%i %i %i", &n, &m, &voltage);
    eee_set_freq(n, m);
    eee_set_voltage(voltage);
}

EEE_PROC_READFUNC(pll) {
    eee_pll_read();
    EEE_PROC_MEMCPY(eee_pll_data, eee_pll_datalen);
}

EEE_PROC_READFUNC(fan_speed) {
    int speed = eee_fan_get_speed();
    EEE_PROC_PRINTF("%d\n", speed);
}

EEE_PROC_WRITEFUNC(fan_speed) {
    unsigned int speed = 0;
    EEE_PROC_SCANF(1, "%u", &speed);
    eee_fan_set_speed(speed);
}

EEE_PROC_READFUNC(fan_rpm) {
    int rpm = eee_fan_get_rpm();
    EEE_PROC_PRINTF("%d\n", rpm);
}

EEE_PROC_READFUNC(fan_manual) {
    EEE_PROC_PRINTF("%d\n", eee_fan_get_manual());
}

EEE_PROC_WRITEFUNC(fan_manual) {
    int manual = 0;
    EEE_PROC_SCANF(1, "%i", &manual);
    eee_fan_set_manual(manual);
}

#if 0
EEE_PROC_READFUNC(fan_mode) {
    enum eee_fan_mode mode = eee_fan_get_mode();
    switch (mode) {
        case Manual:    EEE_PROC_PRINTF("manual\n");
                        break;
        case Automatic: EEE_PROC_PRINTF("auto\n");
                        break;
        case Embedded:  EEE_PROC_PRINTF("embedded\n");
                        break;
    }
}

EEE_PROC_WRITEFUNC(fan_mode) {
    enum eee_fan_mode mode = Automatic;
    char inputstr[16];
    EEE_PROC_SCANF(1, "%15s", inputstr);
    if (strcmp(inputstr, "manual") == 0) {
        mode = Manual;
    } else if (strcmp(inputstr, "auto") == 0) {
        mode = Automatic;
    } else if (strcmp(inputstr, "embedded") == 0) {
        mode = Embedded;
    }
    eee_fan_set_mode(mode);
}
#endif

EEE_PROC_READFUNC(temperature) {
    unsigned int t = eee_get_temperature();
    EEE_PROC_PRINTF("%d\n", t);
}

EEE_PROC_FILES_BEGIN
    EEE_PROC_RW(fsb,            0644),
    EEE_PROC_RO(pll,            0400),
    EEE_PROC_RW(fan_speed,      0644),
    EEE_PROC_RO(fan_rpm,        0444),
    EEE_PROC_RW(fan_manual,     0644),
    EEE_PROC_RO(temperature,    0444),
EEE_PROC_FILES_END
    

int eee_proc_readfunc(char *buffer, char **buffer_location, off_t offset,
                      int buffer_length, int *eof, void *data)
{
    struct eee_proc_file *procfile = (struct eee_proc_file *)data;
    int bufpos = 0;

    if (!procfile || !procfile->readfunc) {
        return -EIO;
    }

    *eof = 1;
    if (offset > 0) {
        return 0;
    }

    (*procfile->readfunc)(buffer, buffer_length, &bufpos);
    return bufpos;
}

int eee_proc_writefunc(struct file *file, const char *buffer,
                       unsigned long count, void *data)
{
    char userdata[129];
    int bufpos = 0;
    struct eee_proc_file *procfile = (struct eee_proc_file *)data;

    if (!procfile || !procfile->writefunc) {
        return -EIO;
    }

    if (copy_from_user(userdata, buffer, (count > 128) ? 128 : count)) {
        printk(KERN_DEBUG "eee: copy_from_user() failed\n");
        return -EIO;
    }
    userdata[128] = 0;      // So that sscanf() doesn't overflow...

    (*procfile->writefunc)(userdata, count, &bufpos);
    return count;
}

int eee_proc_init(void) {
    int i;

    /* Create the /proc/eee directory. */
    eee_proc_rootdir = proc_mkdir("eee", &proc_root);
    if (!eee_proc_rootdir) {
        printk(KERN_ERR "eee: Unable to create /proc/eee\n");
        return false;
    }
    eee_proc_rootdir->owner = THIS_MODULE;

    /* Create the individual proc files. */
    for (i=0; eee_proc_files[i].name; i++) {
        struct proc_dir_entry *proc_file;
        struct eee_proc_file *f = &eee_proc_files[i];

        proc_file = create_proc_entry(f->name, f->mode, eee_proc_rootdir);
        if (!proc_file) {
            printk(KERN_ERR "eee: Unable to create /proc/eee/%s", f->name);
            goto proc_init_cleanup;
        }
        proc_file->read_proc = &eee_proc_readfunc;
        if (f->writefunc) {
            proc_file->write_proc = &eee_proc_writefunc;
        }
        proc_file->data = f;
        proc_file->owner = THIS_MODULE;
        proc_file->mode = S_IFREG | f->mode;
        proc_file->uid = 0;
        proc_file->gid = 0;
    }
    return true;

    /* We had an error, so cleanup all of the proc files... */
proc_init_cleanup:
    for (; i >= 0; i--) {
        remove_proc_entry(eee_proc_files[i].name, eee_proc_rootdir);
    }
    remove_proc_entry("eee", &proc_root);
    return false;
}

void eee_proc_cleanup(void) {
    int i;
    for (i = 0; eee_proc_files[i].name; i++) {
        remove_proc_entry(eee_proc_files[i].name, eee_proc_rootdir);
    }
    remove_proc_entry("eee", &proc_root);
}



/*** Module initialization ***/

int init_module(void) {
    eee_pll_init();
    eee_proc_init();
    printk(KERN_NOTICE "Asus eeePC extras, version %s\n", EEE_VERSION);
    return 0;
}

void cleanup_module(void) {
    eee_pll_cleanup();
    eee_proc_cleanup();
}


