/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_SPI SPI Functions
 * @brief PIOS interface to read and write from SPI ports
 * @{
 *
 * @file       pios_spi.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
 * @author     dRonin, http://dronin.org, Copyright (C) 2016
 * @brief      Hardware Abstraction Layer for SPI ports of STM32
 * @see        The GNU Public License (GPL) Version 3
 * @notes
 *
 * Note that additional chip select lines can be easily added by using
 * the remaining free GPIOs of the core module. Shared SPI ports should be
 * arbitrated with Mutexes to avoid collisions!
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>
 */
#include <pios.h>

#if defined(PIOS_INCLUDE_SPI)

#include <pios_spi_priv.h>

#if !defined(STM32F30X)
#define SPI_SendData8(regs,b) do { (regs)->DR = (b); } while (0)
#define SPI_ReceiveData8(regs) ((regs)->DR)
#endif

typedef enum {
	PIOS_SPI_PRESCALER_2 = 0,
	PIOS_SPI_PRESCALER_4 = 1,
	PIOS_SPI_PRESCALER_8 = 2,
	PIOS_SPI_PRESCALER_16 = 3,
	PIOS_SPI_PRESCALER_32 = 4,
	PIOS_SPI_PRESCALER_64 = 5,
	PIOS_SPI_PRESCALER_128 = 6,
	PIOS_SPI_PRESCALER_256 = 7
} SPIPrescalerTypeDef;


static bool PIOS_SPI_validate(struct pios_spi_dev *com_dev)
{
	/* Should check device magic here */
	return (true);
}

static struct pios_spi_dev *PIOS_SPI_alloc(void)
{
	struct pios_spi_dev *dev = PIOS_malloc_no_dma(sizeof(struct pios_spi_dev));

	if (dev) memset(dev, 0, sizeof(*dev));
	return dev;
}

#if defined(PIOS_INCLUDE_SPIDMA) & defined(STM32F4XX)

inline uint32_t PIOS_SPI_DMAClearFlag(DMA_Stream_TypeDef *s)
{
	switch ((uint32_t)s) {
		default:
		case (uint32_t)DMA1_Stream0:
		case (uint32_t)DMA2_Stream0:
			return DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0 | DMA_FLAG_TEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_FEIF0;
		case (uint32_t)DMA1_Stream1:
		case (uint32_t)DMA2_Stream1:
			return DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_FEIF1;
		case (uint32_t)DMA1_Stream2:
		case (uint32_t)DMA2_Stream2:
			return DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_FEIF2;
		case (uint32_t)DMA1_Stream3:
		case (uint32_t)DMA2_Stream3:
			return DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3;
		case (uint32_t)DMA1_Stream4:
		case (uint32_t)DMA2_Stream4:
			return DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_DMEIF4 | DMA_FLAG_FEIF4;
		case (uint32_t)DMA1_Stream5:
		case (uint32_t)DMA2_Stream5:
			return DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5;
		case (uint32_t)DMA1_Stream6:
		case (uint32_t)DMA2_Stream6:
			return DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6;
		case (uint32_t)DMA1_Stream7:
		case (uint32_t)DMA2_Stream7:
			return DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 | DMA_FLAG_TEIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7;
	}
}

inline uint32_t PIOS_SPI_DMATCFlag(DMA_Stream_TypeDef *s)
{
	switch ((uint32_t)s) {
		default:
		case (uint32_t)DMA1_Stream0:
		case (uint32_t)DMA2_Stream0:
			return DMA_FLAG_TCIF0;
		case (uint32_t)DMA1_Stream1:
		case (uint32_t)DMA2_Stream1:
			return DMA_FLAG_TCIF1;
		case (uint32_t)DMA1_Stream2:
		case (uint32_t)DMA2_Stream2:
			return DMA_FLAG_TCIF2;
		case (uint32_t)DMA1_Stream3:
		case (uint32_t)DMA2_Stream3:
			return DMA_FLAG_TCIF3;
		case (uint32_t)DMA1_Stream4:
		case (uint32_t)DMA2_Stream4:
			return DMA_FLAG_TCIF4;
		case (uint32_t)DMA1_Stream5:
		case (uint32_t)DMA2_Stream5:
			return DMA_FLAG_TCIF5;
		case (uint32_t)DMA1_Stream6:
		case (uint32_t)DMA2_Stream6:
			return DMA_FLAG_TCIF6;
		case (uint32_t)DMA1_Stream7:
		case (uint32_t)DMA2_Stream7:
			return DMA_FLAG_TCIF7;
	}
}

static bool PIOS_SPI_InitDMAStream(DMA_Stream_TypeDef *s, uint32_t channel, uint32_t periph_addr, bool send)
{
	DMA_Cmd(s, DISABLE);
	while (s->CR & DMA_SxCR_EN);

	DMA_DeInit(s);

	DMA_InitTypeDef init;
	DMA_StructInit(&init);

	init.DMA_Channel = channel;

	init.DMA_Memory0BaseAddr = 0;
	init.DMA_PeripheralBaseAddr = periph_addr;
	init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	init.DMA_MemoryInc = DMA_MemoryInc_Enable;
	init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

	init.DMA_DIR = send ? DMA_DIR_MemoryToPeripheral : DMA_DIR_PeripheralToMemory;
	init.DMA_Mode = DMA_Mode_Normal;

	init.DMA_Priority = DMA_Priority_VeryHigh;
	init.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	init.DMA_FIFOMode = DMA_FIFOMode_Disable;
	init.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;

	DMA_Init(s, &init);
	DMA_ITConfig(s, DMA_IT_TC, DISABLE);

	return true;
}

static bool PIOS_SPI_InitializeDMA(const struct pios_spi_cfg *cfg)
{
	if (!cfg) return false;
	if (cfg->dma_send_stream == NULL || cfg->dma_recv_stream == NULL) return false;

	return PIOS_SPI_InitDMAStream(cfg->dma_send_stream, cfg->dma_send_channel, (uint32_t)&cfg->regs->DR, true) &&
		PIOS_SPI_InitDMAStream(cfg->dma_recv_stream, cfg->dma_recv_channel, (uint32_t)&cfg->regs->DR, false);
}

#endif // PIOS_INCLUDE_SPIDMA & STM32F4XX

/**
* Initialises SPI pins
* \param[in] mode currently only mode 0 supported
* \return < 0 if initialisation failed
*/
int32_t PIOS_SPI_Init(pios_spi_t *spi_id, const struct pios_spi_cfg *cfg)
{
	uint32_t	init_ssel = 0;

	PIOS_Assert(spi_id);
	PIOS_Assert(cfg);
	PIOS_Assert(cfg->init.SPI_Mode == SPI_Mode_Master);

	struct pios_spi_dev *spi_dev;

	spi_dev = (struct pios_spi_dev *) PIOS_SPI_alloc();
	if (!spi_dev) goto out_fail;

	/* Bind the configuration to the device instance */
	spi_dev->cfg = cfg;

	spi_dev->busy = PIOS_Semaphore_Create();

	switch (spi_dev->cfg->init.SPI_NSS) {
	case SPI_NSS_Soft:
		/* We're a master in soft NSS mode, make sure we see NSS high at all times. */
		SPI_NSSInternalSoftwareConfig(spi_dev->cfg->regs, SPI_NSSInternalSoft_Set);
		/* Init as many slave selects as the config advertises. */
		init_ssel = spi_dev->cfg->slave_count;
		break;

	case SPI_NSS_Hard:
		/* only legal for single-slave config */
		PIOS_Assert(spi_dev->cfg->slave_count == 1);
		init_ssel = 1;
		SPI_SSOutputCmd(spi_dev->cfg->regs, ENABLE);
		break;

	default:
		PIOS_Assert(0);
	}

#ifndef STM32F10X_MD
	/* Initialize the GPIO pins */
	/* note __builtin_ctz() due to the difference between GPIO_PinX and GPIO_PinSourceX */
	if (spi_dev->cfg->remap) {
		GPIO_PinAFConfig(spi_dev->cfg->sclk.gpio,
		                 __builtin_ctz(spi_dev->cfg->sclk.init.GPIO_Pin),
		                 spi_dev->cfg->remap);
		GPIO_PinAFConfig(spi_dev->cfg->mosi.gpio,
		                 __builtin_ctz(spi_dev->cfg->mosi.init.GPIO_Pin),
		                 spi_dev->cfg->remap);
		GPIO_PinAFConfig(spi_dev->cfg->miso.gpio,
		                 __builtin_ctz(spi_dev->cfg->miso.init.GPIO_Pin),
		                 spi_dev->cfg->remap);

		/* Only remap pins to SPI function if hardware slave select is
		 * used. */
		if (spi_dev->cfg->init.SPI_NSS == SPI_NSS_Hard) {
			for (uint32_t i = 0; i < init_ssel; i++) {
				GPIO_PinAFConfig(spi_dev->cfg->ssel[i].gpio,
						 __builtin_ctz(spi_dev->cfg->ssel[i].init.GPIO_Pin),
						 spi_dev->cfg->remap);
			}
		}
	}
#endif
	GPIO_Init(spi_dev->cfg->sclk.gpio, (GPIO_InitTypeDef *) & (spi_dev->cfg->sclk.init));
	GPIO_Init(spi_dev->cfg->mosi.gpio, (GPIO_InitTypeDef *) & (spi_dev->cfg->mosi.init));
	GPIO_Init(spi_dev->cfg->miso.gpio, (GPIO_InitTypeDef *) & (spi_dev->cfg->miso.init));

	for (uint32_t i = 0; i < init_ssel; i++) {
		/* Since we're driving the SSEL pin in software, ensure that the slave is deselected */
		GPIO_SetBits(spi_dev->cfg->ssel[i].gpio, spi_dev->cfg->ssel[i].init.GPIO_Pin);
		GPIO_Init(spi_dev->cfg->ssel[i].gpio, (GPIO_InitTypeDef *) & (spi_dev->cfg->ssel[i].init));
	}

	/* Initialize the SPI block */
	SPI_I2S_DeInit(spi_dev->cfg->regs);
	SPI_Init(spi_dev->cfg->regs, (SPI_InitTypeDef *) & (spi_dev->cfg->init));
	SPI_CalculateCRC(spi_dev->cfg->regs, DISABLE);

#if defined(STM32F30X)
	/* Configure the RX FIFO Threshold -- 8 bits */
	SPI_RxFIFOThresholdConfig(spi_dev->cfg->regs, SPI_RxFIFOThreshold_QF);
#endif

#if defined(PIOS_INCLUDE_SPIDMA) & defined(STM32F4XX)
	spi_dev->dma_initialized = PIOS_SPI_InitializeDMA(spi_dev->cfg);
#endif

	/* Enable SPI */
	SPI_Cmd(spi_dev->cfg->regs, ENABLE);

	*spi_id = spi_dev;

	return (0);

out_fail:
	return (-1);
}

int32_t PIOS_SPI_SetClockSpeed(pios_spi_t spi_dev, uint32_t spi_speed)
{
	bool valid = PIOS_SPI_validate(spi_dev);
	PIOS_Assert(valid);

	SPI_InitTypeDef SPI_InitStructure;

	SPIPrescalerTypeDef spi_prescaler;

	//SPI clock is different depending on the bus
	uint32_t spiBusClock = PIOS_SYSCLK;

#if defined(STM32F40_41xxx) || defined(STM32F446xx)
	if(spi_dev->cfg->regs == SPI1)
		spiBusClock = PIOS_SYSCLK / 2;
	else
		spiBusClock = PIOS_SYSCLK / 4;
#elif defined(STM32F30X)
	// SPI1 runs half as a fast
	if(spi_dev->cfg->regs == SPI1)
		spiBusClock = PIOS_SYSCLK / 2;
#elif !defined(STM32F10X_MD)
	// F1 not handled explicitly --- spiBusClock is right above
#error Unrecognized architecture
#endif

	//The needed prescaler for desired speed
	float desiredPrescaler=(float) spiBusClock / spi_speed;

	//Choosing the existing prescaler nearest the desiredPrescaler
	if(desiredPrescaler <= 2)
		spi_prescaler = PIOS_SPI_PRESCALER_2;
	else if(desiredPrescaler <= 4)
		spi_prescaler = PIOS_SPI_PRESCALER_4;
	else if(desiredPrescaler <= 8)
		spi_prescaler = PIOS_SPI_PRESCALER_8;
	else if(desiredPrescaler <= 16)
		spi_prescaler = PIOS_SPI_PRESCALER_16;
	else if(desiredPrescaler <= 32)
		spi_prescaler = PIOS_SPI_PRESCALER_32;
	else if(desiredPrescaler <= 64)
		spi_prescaler = PIOS_SPI_PRESCALER_64;
	else if(desiredPrescaler <= 128)
		spi_prescaler = PIOS_SPI_PRESCALER_128;
	else
		spi_prescaler = PIOS_SPI_PRESCALER_256;

	/* Start with a copy of the default configuration for the peripheral */
	SPI_InitStructure = spi_dev->cfg->init;

	/* Adjust the prescaler for the peripheral's clock */
	SPI_InitStructure.SPI_BaudRatePrescaler = ((uint16_t) spi_prescaler & 7) << 3;

	/* Write back the new configuration */
	SPI_Init(spi_dev->cfg->regs, &SPI_InitStructure);

	//return set speed
	return spiBusClock >> (1 + spi_prescaler);
}

int32_t PIOS_SPI_ClaimBus(pios_spi_t spi_dev)
{
	bool valid = PIOS_SPI_validate(spi_dev);
	PIOS_Assert(valid)

	if (PIOS_Semaphore_Take(spi_dev->busy, PIOS_SEMAPHORE_TIMEOUT_MAX) != true)
		return -1;

	return 0;
}

int32_t PIOS_SPI_ReleaseBus(pios_spi_t spi_dev)
{
	bool valid = PIOS_SPI_validate(spi_dev);
	PIOS_Assert(valid)

	PIOS_Semaphore_Give(spi_dev->busy);

	return 0;
}

int32_t PIOS_SPI_RC_PinSet(pios_spi_t spi_dev, uint32_t slave_id, bool pin_value)
{
	bool valid = PIOS_SPI_validate(spi_dev);
	PIOS_Assert(valid)
	PIOS_Assert(slave_id <= spi_dev->cfg->slave_count)

	if (pin_value) {
		GPIO_SetBits(spi_dev->cfg->ssel[slave_id].gpio, spi_dev->cfg->ssel[slave_id].init.GPIO_Pin);
	} else {
		GPIO_ResetBits(spi_dev->cfg->ssel[slave_id].gpio, spi_dev->cfg->ssel[slave_id].init.GPIO_Pin);
	}

	return 0;
}

uint8_t PIOS_SPI_TransferByte(pios_spi_t spi_dev, uint8_t b)
{
	bool valid = PIOS_SPI_validate(spi_dev);
	PIOS_Assert(valid)

	uint8_t rx_byte;

	/*
	 * Procedure taken from STM32F10xxx Reference Manual section 23.3.5
	 */

	/* Make sure the RXNE flag is cleared by reading the DR register */
	SPI_ReceiveData8(spi_dev->cfg->regs);

	/* Wait until the TXE goes high */
	while (SPI_I2S_GetFlagStatus(spi_dev->cfg->regs, SPI_I2S_FLAG_TXE) == RESET);

	/* Start the transfer */
	SPI_SendData8(spi_dev->cfg->regs, b);

	/* Wait until there is a byte to read */
	while (SPI_I2S_GetFlagStatus(spi_dev->cfg->regs, SPI_I2S_FLAG_RXNE) == RESET);

	/* Read the rx'd byte */
	rx_byte = SPI_ReceiveData8(spi_dev->cfg->regs);

	/* Wait until the TXE goes high */
	while (!(spi_dev->cfg->regs->SR & SPI_I2S_FLAG_TXE));

	/* Wait for SPI transfer to have fully completed */
	while (spi_dev->cfg->regs->SR & SPI_I2S_FLAG_BSY);

	/* Return received byte */
	return rx_byte;
}

/**
* Transfers a block of bytes via PIO.
*
* \param[in] spi_id SPI device handle
* \param[in] send_buffer pointer to buffer which should be sent.<BR>
* If NULL, 0xff (all-one) will be sent.
* \param[in] receive_buffer pointer to buffer which should get the received values.<BR>
* If NULL, received bytes will be discarded.
* \param[in] len number of bytes which should be transfered
* \return >= 0 if no error during transfer
* \return -1 if disabled SPI port selected
*/
static int32_t SPI_PIO_TransferBlock(pios_spi_t spi_dev,
		const uint8_t *send_buffer, uint8_t *receive_buffer, uint16_t len)
{
	uint8_t b;

	bool valid = PIOS_SPI_validate(spi_dev);
	PIOS_Assert(valid)

#if defined(PIOS_INCLUDE_SPIDMA) & defined(STM32F4XX)
	if (spi_dev->dma_initialized & (len > 1)) {
		const struct pios_spi_cfg *cfg = spi_dev->cfg;
		uint8_t dummy_rx = 0, dummy_tx = 0xff;

		if (send_buffer) {
			cfg->dma_send_stream->CR |= DMA_MemoryInc_Enable;
			DMA_MemoryTargetConfig(cfg->dma_send_stream, (uint32_t)send_buffer, DMA_Memory_0);
		} else {
			cfg->dma_send_stream->CR &= ~DMA_MemoryInc_Enable;
			DMA_MemoryTargetConfig(cfg->dma_send_stream, (uint32_t)&dummy_tx, DMA_Memory_0);
		}

		if (receive_buffer) {
			cfg->dma_recv_stream->CR |= DMA_MemoryInc_Enable;
			DMA_MemoryTargetConfig(cfg->dma_recv_stream, (uint32_t)receive_buffer, DMA_Memory_0);
		} else {
			cfg->dma_recv_stream->CR &= ~DMA_MemoryInc_Enable;
			DMA_MemoryTargetConfig(cfg->dma_recv_stream, (uint32_t)&dummy_rx, DMA_Memory_0);
		}

		DMA_SetCurrDataCounter(cfg->dma_send_stream, len);
		DMA_SetCurrDataCounter(cfg->dma_recv_stream, len);

		DMA_ClearFlag(cfg->dma_send_stream, PIOS_SPI_DMATCFlag(cfg->dma_send_stream));
		DMA_ClearFlag(cfg->dma_recv_stream, PIOS_SPI_DMATCFlag(cfg->dma_recv_stream));

		DMA_Cmd(cfg->dma_send_stream, ENABLE);
		DMA_Cmd(cfg->dma_recv_stream, ENABLE);

		SPI_I2S_DMACmd(cfg->regs, SPI_I2S_DMAReq_Tx, ENABLE);
		SPI_I2S_DMACmd(cfg->regs, SPI_I2S_DMAReq_Rx, ENABLE);

		SPI_Cmd(cfg->regs, ENABLE);

		while (DMA_GetFlagStatus(cfg->dma_recv_stream, PIOS_SPI_DMATCFlag(cfg->dma_recv_stream)) == RESET);
		//while (DMA_GetFlagStatus(cfg->dma_send_stream, PIOS_SPI_DMATCFlag(cfg->dma_send_stream)) == RESET);

		SPI_Cmd(cfg->regs, DISABLE);
		
		SPI_I2S_DMACmd(cfg->regs, SPI_I2S_DMAReq_Tx, DISABLE);
		SPI_I2S_DMACmd(cfg->regs, SPI_I2S_DMAReq_Rx, DISABLE);

		DMA_Cmd(cfg->dma_send_stream, DISABLE);
		DMA_Cmd(cfg->dma_recv_stream, DISABLE);

		while (cfg->dma_send_stream->CR & DMA_SxCR_EN);
		while (cfg->dma_recv_stream->CR & DMA_SxCR_EN);

		return 0;
	}
#endif

	while (len--) {
		/* get the byte to send */
		b = send_buffer ? *(send_buffer++) : 0xff;

		/* Wait until the TXE goes high */
		while (SPI_I2S_GetFlagStatus(spi_dev->cfg->regs, SPI_I2S_FLAG_TXE) == RESET);

		/* Start the transfer */
		SPI_SendData8(spi_dev->cfg->regs, b);

		/* Wait until there is a byte to read */
		while (SPI_I2S_GetFlagStatus(spi_dev->cfg->regs, SPI_I2S_FLAG_RXNE) == RESET);

		/* Read the rx'd byte */
		b = SPI_ReceiveData8(spi_dev->cfg->regs);

		/* save the received byte */
		if (receive_buffer)
			*(receive_buffer++) = b;
	}

	/* Wait for SPI transfer to have fully completed */
	while (spi_dev->cfg->regs->SR & SPI_I2S_FLAG_BSY);

	return 0;
}

int32_t PIOS_SPI_TransferBlock(pios_spi_t spi_dev, const uint8_t *send_buffer,
		uint8_t *receive_buffer, uint16_t len)
{
	return SPI_PIO_TransferBlock(spi_dev, send_buffer, receive_buffer, len);
}

#endif

/**
  * @}
  * @}
  */
