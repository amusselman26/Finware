// Minimal FatFs mount diagnostic for STM32 (STM32Duino core)
#include <Arduino.h>
#include <STM32SD.h>

// Pull in FatFs types
extern "C" {
  #include "ff.h"          // FRESULT, FATFS, f_mount, f_opendir, f_readdir, etc.
  #include "ff_gen_drv.h"  // FATFS_LinkDriver
  // The STM32 core provides an SD disk I/O driver. These names are common:
  DSTATUS SD_initialize(BYTE);     // declared in sd_diskio.c
  DRESULT SD_read(BYTE*, LBA_t, UINT);
  DRESULT SD_write(const BYTE*, LBA_t, UINT);
  DRESULT SD_ioctl(BYTE, void*);
}

// These are provided by STM32 core’s SD driver glue:
extern "C" int FATFS_LinkDriver(const Diskio_drvTypeDef *drv, char *path);
extern "C" const Diskio_drvTypeDef SD_Driver;

// Global FatFs objects
static FATFS fs;          // Work area (filesystem object)
static char  sdPath[4];   // Logical drive path like "0:"
static bool  linked = false;

static const __FlashStringHelper* frName(FRESULT fr) {
  switch (fr) {
    case FR_OK:                  return F("FR_OK");
    case FR_DISK_ERR:            return F("FR_DISK_ERR");
    case FR_INT_ERR:             return F("FR_INT_ERR");
    case FR_NOT_READY:           return F("FR_NOT_READY");
    case FR_NO_FILE:             return F("FR_NO_FILE");
    case FR_NO_PATH:             return F("FR_NO_PATH");
    case FR_INVALID_NAME:        return F("FR_INVALID_NAME");
    case FR_DENIED:              return F("FR_DENIED");
    case FR_EXIST:               return F("FR_EXIST");
    case FR_INVALID_OBJECT:      return F("FR_INVALID_OBJECT");
    case FR_WRITE_PROTECTED:     return F("FR_WRITE_PROTECTED");
    case FR_INVALID_DRIVE:       return F("FR_INVALID_DRIVE");
    case FR_NOT_ENABLED:         return F("FR_NOT_ENABLED");
    case FR_NO_FILESYSTEM:       return F("FR_NO_FILESYSTEM");
    case FR_MKFS_ABORTED:        return F("FR_MKFS_ABORTED");
    case FR_TIMEOUT:             return F("FR_TIMEOUT");
    case FR_LOCKED:              return F("FR_LOCKED");
    case FR_NOT_ENOUGH_CORE:     return F("FR_NOT_ENOUGH_CORE");
    case FR_TOO_MANY_OPEN_FILES: return F("FR_TOO_MANY_OPEN_FILES");
    case FR_INVALID_PARAMETER:   return F("FR_INVALID_PARAMETER");
    default:                     return F("FR_???");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println(F("\n[SD DIAG] Starting…"));

  // If your slot has no detect pin, force ignore
  // SD.begin(SD_DETECT_NONE) only affects STM32SD’s wrapper; we go lower-level here.

  // 1) Link SD disk driver -> sdPath becomes something like "0:"
  if (!linked) {
    if (FATFS_LinkDriver(&SD_Driver, sdPath) == 0) {
      linked = true;
      Serial.print(F("[SD DIAG] FATFS_LinkDriver OK. sdPath="));
      Serial.println(sdPath);
    } else {
      Serial.println(F("[SD DIAG] FATFS_LinkDriver FAILED"));
      return;
    }
  }

  // 2) Try to mount immediately
  FRESULT fr = f_mount(&fs, sdPath, 1);  // 1 = mount now
  Serial.print(F("[SD DIAG] f_mount -> ")); Serial.print((int)fr);
  Serial.print(F(" (")); Serial.print(frName(fr)); Serial.println(F(")"));

  if (fr != FR_OK) {
    Serial.println(F("[SD DIAG] Mount failed. Common causes:"));
    Serial.println(F(" - FR_NO_FILESYSTEM: bad/odd partition; reformat FAT32 with 1 primary partition"));
    Serial.println(F(" - FR_NOT_READY: card not detected/initialized; try SD_DETECT_NONE or reseat"));
    Serial.println(F(" - FR_DISK_ERR: electrical/card fault; try another card"));
    return;
  }

  // 3) List root to prove it’s mounted
  DIR dir;
  FILINFO fno;
  fr = f_opendir(&dir, sdPath);  // open root
  if (fr != FR_OK) {
    Serial.print(F("[SD DIAG] f_opendir failed: "));
    Serial.print((int)fr); Serial.print(F(" (")); Serial.print(frName(fr)); Serial.println(F(")"));
  } else {
    Serial.println(F("[SD DIAG] Root listing:"));
    for (;;) {
      fr = f_readdir(&dir, &fno);
      if (fr != FR_OK || fno.fname[0] == 0) break;
      Serial.print(fno.fname);
      if (fno.fattrib & AM_DIR) Serial.println(F("/"));
      else {
        Serial.print(F("\t")); Serial.println((uint32_t)fno.fsize);
      }
    }
    f_closedir(&dir);
  }

  // 4) Unmount cleanly
  fr = f_mount(nullptr, sdPath, 0);
  Serial.print(F("[SD DIAG] f_unmount -> ")); Serial.print((int)fr);
  Serial.print(F(" (")); Serial.print(frName(fr)); Serial.println(F(")"));

  Serial.println(F("[SD DIAG] Done."));
}

void loop() {}
