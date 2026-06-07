/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   SPI-mode SD card diskio driver for FatFs.
  *
  *   Hardware: SPI1 (125 kHz init → 4 MHz after init)
  *             PA4 = SD_CS (software GPIO, active-low)
  *             SPI Mode 0 (CPOL=0, CPHA=0)
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "ff_gen_drv.h"
#include "diskio.h"
#include "main.h"
#include "user_diskio.h"
#include "uart_debug.h"

/* ── SD card type flags ──────────────────────────────────────────────────── */
#define CT_MMC   0x01
#define CT_SD1   0x02   /* SDSC v1 */
#define CT_SD2   0x04   /* SDSC v2 or SDHC/SDXC */
#define CT_SDHC  0x08   /* SDHC / SDXC (block-addressed) */

/* ── SD commands ─────────────────────────────────────────────────────────── */
#define CMD0   0    /* GO_IDLE_STATE      */
#define CMD1   1    /* SEND_OP_COND       */
#define CMD8   8    /* SEND_IF_COND       */
#define CMD9   9    /* SEND_CSD           */
#define CMD12  12   /* STOP_TRANSMISSION  */
#define CMD16  16   /* SET_BLOCKLEN       */
#define CMD17  17   /* READ_SINGLE_BLOCK  */
#define CMD18  18   /* READ_MULTIPLE_BLOCK*/
#define CMD24  24   /* WRITE_BLOCK        */
#define CMD25  25   /* WRITE_MULTIPLE_BLOCK*/
#define CMD41  41   /* SD_SEND_OP_COND    */
#define CMD55  55   /* APP_CMD            */
#define CMD58  58   /* READ_OCR           */

#define SD_DUMMY_BYTE              0xFFU
#define SD_DATA_TOKEN              0xFEU
#define SD_INIT_TIMEOUT_MS         1000U
#define SD_READY_TIMEOUT_MS        500U
#define SD_BLOCK_READ_TIMEOUT_MS   200U

/* ── Private state ───────────────────────────────────────────────────────── */
static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType = 0;
static volatile uint8_t g_spi_fault = 0U;

extern SPI_HandleTypeDef hspi1;

static int sd_wait_ready(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  do {
    if (sd_spi_xchg(SD_DUMMY_BYTE) == SD_DUMMY_BYTE) {
      return 1;
    }
  } while ((HAL_GetTick() - start) < timeout_ms);

  return 0;
}

void USER_disk_reset(void)
{
  Stat = STA_NOINIT;
  CardType = 0U;
  g_spi_fault = 0U;
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

void sd_spi_set_prescaler(uint32_t prescaler)
{
  hspi1.Init.BaudRatePrescaler = prescaler;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    g_spi_fault = 1U;
  }
}

void sd_deselect(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  (void)sd_spi_xchg(SD_DUMMY_BYTE);
}

int sd_select(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
  (void)sd_spi_xchg(SD_DUMMY_BYTE);

  if (sd_wait_ready(SD_READY_TIMEOUT_MS)) {
    return 1;
  }

  sd_deselect();
  return 0;
}

uint8_t sd_spi_xchg(uint8_t tx)
{
  uint8_t rx = SD_DUMMY_BYTE;

  if (HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1U, 100U) != HAL_OK) {
    g_spi_fault = 1U;
    return SD_DUMMY_BYTE;
  }

  return rx;
}

void sd_spi_recv(uint8_t *buff, UINT len)
{
  if (buff == NULL || len == 0U) return;

  for (UINT i = 0U; i < len; ++i) {
    buff[i] = sd_spi_xchg(SD_DUMMY_BYTE);
  }
}

uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg)
{
  uint8_t buf[6];
  uint8_t crc = 0x01U;
  uint8_t res;
  uint8_t n;

  if (cmd & 0x80U) {
    cmd &= 0x7FU;
    res = sd_send_cmd(CMD55, 0U);
    if (res > 1U) {
      return res;
    }
  }

  sd_deselect();
  if (!sd_select()) {
    return 0xFFU;
  }

  buf[0] = (uint8_t)(0x40U | cmd);
  buf[1] = (uint8_t)(arg >> 24);
  buf[2] = (uint8_t)(arg >> 16);
  buf[3] = (uint8_t)(arg >> 8);
  buf[4] = (uint8_t)arg;

  if (cmd == CMD0) {
    crc = 0x95U;
  } else if (cmd == CMD8) {
    crc = 0x87U;
  }
  buf[5] = crc;

  for (n = 0U; n < sizeof(buf); ++n) {
    (void)sd_spi_xchg(buf[n]);
  }

  if (cmd == CMD12) {
    (void)sd_spi_xchg(SD_DUMMY_BYTE);
  }

  for (n = 10U; n > 0U; --n) {
    res = sd_spi_xchg(SD_DUMMY_BYTE);
    if ((res & 0x80U) == 0U) {
      return res;
    }
  }

  return 0xFFU;
}

int sd_recv_datablock(uint8_t *buff, UINT len)
{
  uint8_t token;
  uint32_t start = HAL_GetTick();

  if (buff == NULL || len == 0U) return 0;

  do {
    token = sd_spi_xchg(SD_DUMMY_BYTE);
    if (token == SD_DATA_TOKEN) {
      break;
    }
  } while ((HAL_GetTick() - start) < SD_BLOCK_READ_TIMEOUT_MS);

  if (token != SD_DATA_TOKEN) {
    return 0;
  }

  sd_spi_recv(buff, len);
  (void)sd_spi_xchg(SD_DUMMY_BYTE);
  (void)sd_spi_xchg(SD_DUMMY_BYTE);
  return 1;
}

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
    uint8_t ty = 0U;
    uint8_t ocr[4];
    uint32_t start;

    if (pdrv != 0U) {
      return STA_NOINIT;
    }

    USER_disk_reset();
    sd_spi_set_prescaler(SPI_BAUDRATEPRESCALER_128);

    sd_deselect();
    for (uint8_t i = 0U; i < 10U; ++i) {
      sd_spi_xchg(SD_DUMMY_BYTE);
    }

    if (sd_send_cmd(CMD0, 0U) == 1U) {
      start = HAL_GetTick();
      if (sd_send_cmd(CMD8, 0x1AAU) == 1U) {
        sd_spi_recv(ocr, 4U);
        if ((ocr[2] == 0x01U) && (ocr[3] == 0xAAU)) {
          while ((HAL_GetTick() - start) < SD_INIT_TIMEOUT_MS) {
            if (sd_send_cmd((uint8_t)(CMD41 | 0x80U), 1UL << 30) == 0U) {
              break;
            }
            HAL_Delay(1U);
          }

          if ((HAL_GetTick() - start) < SD_INIT_TIMEOUT_MS &&
              sd_send_cmd(CMD58, 0U) == 0U) {
            sd_spi_recv(ocr, 4U);
            ty = (ocr[0] & 0x40U) ? (CT_SD2 | CT_SDHC) : CT_SD2;
          }
        }
      } else {
        uint8_t cmd = sd_send_cmd((uint8_t)(CMD41 | 0x80U), 0U) <= 1U ? (uint8_t)(CMD41 | 0x80U) : CMD1;
        ty = (cmd == (uint8_t)(CMD41 | 0x80U)) ? CT_SD1 : CT_MMC;

        while ((HAL_GetTick() - start) < SD_INIT_TIMEOUT_MS) {
          if (sd_send_cmd(cmd, 0U) == 0U) {
            break;
          }
          HAL_Delay(1U);
        }

        if ((HAL_GetTick() - start) >= SD_INIT_TIMEOUT_MS || sd_send_cmd(CMD16, 512U) != 0U) {
          ty = 0U;
        }
      }
    }

    CardType = ty;
    sd_deselect();

    if ((ty != 0U) && (g_spi_fault == 0U)) {
      Stat &= (DSTATUS)~STA_NOINIT;
      sd_spi_set_prescaler(SPI_BAUDRATEPRESCALER_4);
    } else {
      USER_disk_reset();
    }

    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    if (pdrv != 0U) {
      return STA_NOINIT;
    }

    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
    if ((pdrv != 0U) || (count == 0U) || (buff == NULL)) {
      return RES_PARERR;
    }

    if (Stat & STA_NOINIT) {
      return RES_NOTRDY;
    }

    for (UINT i = 0U; i < count; ++i) {
      DWORD addr = sector + i;

      if ((CardType & CT_SDHC) == 0U) {
        addr *= 512UL;
      }

      if (sd_send_cmd(CMD17, addr) != 0U || !sd_recv_datablock(buff + (i * 512U), 512U)) {
        sd_deselect();
        return RES_ERROR;
      }

      sd_deselect();
    }

    return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
    (void)pdrv;
    (void)buff;
    (void)sector;
    (void)count;
    return RES_WRPRT;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    uint8_t csd[16];

    if (pdrv != 0U) {
      return RES_PARERR;
    }

    if (Stat & STA_NOINIT) {
      return RES_NOTRDY;
    }

    switch (cmd) {
    case CTRL_SYNC:
      return sd_select() ? (sd_deselect(), RES_OK) : RES_ERROR;

    case GET_SECTOR_SIZE:
      *(WORD *)buff = 512U;
      return RES_OK;

    case GET_BLOCK_SIZE:
      *(DWORD *)buff = 1U;
      return RES_OK;

    case MMC_GET_TYPE:
      *(BYTE *)buff = CardType;
      return RES_OK;

    case GET_SECTOR_COUNT:
      if (sd_send_cmd(CMD9, 0U) != 0U || !sd_recv_datablock(csd, sizeof(csd))) {
        sd_deselect();
        return RES_ERROR;
      }
      sd_deselect();

      if ((csd[0] >> 6) == 1U) {
        uint32_t csize = ((uint32_t)(csd[7] & 0x3FU) << 16) |
                         ((uint32_t)csd[8] << 8) |
                         csd[9];
        *(DWORD *)buff = (csize + 1UL) << 10;
      } else {
        uint32_t n = (uint32_t)(csd[5] & 0x0FU) +
                     (uint32_t)((csd[10] & 0x80U) >> 7) +
                     (uint32_t)((csd[9] & 0x03U) << 1) + 2UL;
        uint32_t csize = ((uint32_t)(csd[8] & 0xC0U) >> 6) |
                         ((uint32_t)csd[7] << 2) |
                         ((uint32_t)(csd[6] & 0x03U) << 10);
        *(DWORD *)buff = (csize + 1UL) << (n - 9UL);
      }
      return RES_OK;

    default:
      return RES_PARERR;
    }
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

