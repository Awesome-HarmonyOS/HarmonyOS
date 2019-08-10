/*----------------------------------------------------------------------------
 * Copyright (c) <2016-2018>, <Huawei Technologies Co., Ltd>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific prior written
 * permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
 * Notice of Export Control Law
 * ===============================================
 * Huawei LiteOS may be subject to applicable export control laws and regulations, which might
 * include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
 * applicable export control laws and regulations.
 *---------------------------------------------------------------------------*/
#include "enc28j60.h"
#include <stdio.h>

#include <stdio.h>
#include "fsl_debug_console.h"
#include "fsl_spi.h"
#include "board.h"

#include "pin_mux.h"
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_SPI_MASTER_BASEADDR SPI3
#define EXAMPLE_SPI_MASTER_CLK_FREQ CLOCK_GetFreq(kCLOCK_Flexcomm3)
#define EXAMPLE_SPI_MASTER_SSEL kSPI_Ssel2
#define EXAMPLE_SPI_MASTER_SPOL kSPI_SpolActiveAllLow
//port for the enc28j60
void SPI_Init(void)
{
    uint32_t srcFreq = 0;
    /* Transfer structure for half-duplex. */
    spi_master_config_t masterConfig;

    /* Master config */
    SPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.sselNum = EXAMPLE_SPI_MASTER_SSEL;
    masterConfig.sselPol = EXAMPLE_SPI_MASTER_SPOL;
    srcFreq = EXAMPLE_SPI_MASTER_CLK_FREQ;
    SPI_MasterInit(EXAMPLE_SPI_MASTER_BASEADDR, &masterConfig, srcFreq);
    return;
}

extern uint8_t SPI_MasterTransferBlockingByte(SPI_Type *base, spi_transfer_t *xfer);

//SPI1 read or write one byte
unsigned char SPI_ReadWrite(unsigned char writedat,int end)
{
    unsigned char v8_w;
    unsigned char v8_r;
	spi_transfer_t   transfer;
    v8_w = writedat;
    transfer.txData = &v8_w;
    transfer.rxData = &v8_r;
    transfer.dataSize = 1;
    if(end)
    {
        transfer.configFlags = kSPI_FrameAssert;
    }
    else
    {
        transfer.configFlags = kSPI_FrameDelay;
    }
    //LOS_IntLock();
    //SPI_MasterTransferBlockingByte(EXAMPLE_SPI_MASTER_BASEADDR,&transfer);
    v8_r =SPI_MasterTransferBlockingByte(EXAMPLE_SPI_MASTER_BASEADDR,&transfer);    
	//LOS_IntUnLock();
    return  v8_r;
}
 
static unsigned char Enc28j60Bank;
static unsigned int NextPacketPtr;

unsigned char enc28j60ReadOp(unsigned char op, unsigned char address){
	unsigned char dat = 0;	
	dat = op | (address & ADDR_MASK);
	SPI_ReadWrite(dat,0);
    if(address & 0x80)
    {
        SPI_ReadWrite(0xFF,0);
		dat = SPI_ReadWrite(0xFF,1);
    }
    else
    {
		dat = SPI_ReadWrite(0xFF,1);
    }
	return dat;  
}

void enc28j60WriteOp(unsigned char op, unsigned char address, unsigned char data)
{
	unsigned char dat = 0;
	  
	// issue write command
	dat = op | (address & ADDR_MASK);
	SPI_ReadWrite(dat,0);
	// write data
	dat = data;
	SPI_ReadWrite(dat,1);
}

void enc28j60ReadBuffer(unsigned int len, unsigned char* data)
{
	// issue read command
	SPI_ReadWrite(ENC28J60_READ_BUF_MEM,0);
	while(len)
	{
	    len--;
        // read data
        if(len ==0)
        {
            *data = SPI_ReadWrite(0xFF,1);
        }
        else
        {
            *data = SPI_ReadWrite(0xFF,0);
        }
        
        data++;
	}
}

void enc28j60WriteBuffer(unsigned int len, unsigned char* data)
{
	// issue write command
	SPI_ReadWrite(ENC28J60_WRITE_BUF_MEM,0);
	
	while(len)
	{
		len--;
        if(len == 0)
        {
            SPI_ReadWrite(*data,1);
        }
        else
        {
            SPI_ReadWrite(*data,0);
        }
		data++;
	}
}

void enc28j60SetBank(unsigned char address)
{
	// set the bank (if needed)
	if((address & BANK_MASK) != Enc28j60Bank)
    {
        // set the bank
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
        Enc28j60Bank = (address & BANK_MASK);
	}
}

unsigned char enc28j60Read(unsigned char address)
{
	// set the bank
	enc28j60SetBank(address);
	// do the read
	return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}

void enc28j60Write(unsigned char address, unsigned char data)
{
	// set the bank
	enc28j60SetBank(address);
	// do the write
	enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

void enc28j60PhyWrite(unsigned char address, unsigned int data)
{
	// set the PHY register address
	enc28j60Write(MIREGADR, address);
	// write the PHY data
	enc28j60Write(MIWRL, data);
	enc28j60Write(MIWRH, data>>8);
	// wait until the PHY write completes
	while(enc28j60Read(MISTAT) & MISTAT_BUSY)
	{
	    	//Del_10us(1);
			//_nop_();
	}
}

void enc28j60clkout(unsigned char clk)
{
    //setup clkout: 2 is 12.5MHz:
	enc28j60Write(ECOCON, clk & 0x7);
}

void enc28j60Init(unsigned char* macaddr)
{
    SPI_Init();
	// initialize I/O
	//enc28j60CSinit();

    //enc28j60SetSCK();
    //enc28j60HWreset();
	// perform system reset
	enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
//	Del_1ms(250);
	// check CLKRDY bit to see if reset is complete
        // The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
	//while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));
	// do bank 0 stuff
	// initialize receive buffer
	// 16-bit transfers, must write low byte first
	// set receive buffer start address
	NextPacketPtr = RXSTART_INIT;
        // Rx start
	enc28j60Write(ERXSTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXSTH, RXSTART_INIT>>8);
	// set receive pointer address
	enc28j60Write(ERXRDPTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXRDPTH, RXSTART_INIT>>8);
	// RX end
	enc28j60Write(ERXNDL, RXSTOP_INIT&0xFF);
	enc28j60Write(ERXNDH, RXSTOP_INIT>>8);
	// TX start
	enc28j60Write(ETXSTL, TXSTART_INIT&0xFF);
	enc28j60Write(ETXSTH, TXSTART_INIT>>8);
	// TX end
	enc28j60Write(ETXNDL, TXSTOP_INIT&0xFF);
	enc28j60Write(ETXNDH, TXSTOP_INIT>>8);
	// do bank 1 stuff, packet filter:
        // For broadcast packets we allow only ARP packtets
        // All other packets should be unicast only for our mac (MAADR)
        //
        // The pattern to match on is therefore
        // Type     ETH.DST
        // ARP      BROADCAST
        // 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
        // in binary these poitions are:11 0000 0011 1111
        // This is hex 303F->EPMM0=0x3f,EPMM1=0x30
	enc28j60Write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
	enc28j60Write(EPMM0, 0x3f);
	enc28j60Write(EPMM1, 0x30);
	enc28j60Write(EPMCSL, 0xf9);
	enc28j60Write(EPMCSH, 0xf7);
        //
        //
	// do bank 2 stuff
	// enable MAC receive
	enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
	// bring MAC out of reset
	enc28j60Write(MACON2, 0x00);
	// enable automatic padding to 60bytes and CRC operations
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN|MACON3_FULDPX);
	// set inter-frame gap (non-back-to-back)
	enc28j60Write(MAIPGL, 0x12);
	enc28j60Write(MAIPGH, 0x0C);
	// set inter-frame gap (back-to-back)
	enc28j60Write(MABBIPG, 0x12);
	// Set the maximum packet size which the controller will accept
        // Do not send packets longer than MAX_FRAMELEN:
	enc28j60Write(MAMXFLL, MAX_FRAMELEN&0xFF);	
	enc28j60Write(MAMXFLH, MAX_FRAMELEN>>8);
	// do bank 3 stuff
	// write MAC address
	// NOTE: MAC address in ENC28J60 is byte-backward
	enc28j60Write(MAADR5, macaddr[0]);	
	enc28j60Write(MAADR4, macaddr[1]);
	enc28j60Write(MAADR3, macaddr[2]);
	enc28j60Write(MAADR2, macaddr[3]);
	enc28j60Write(MAADR1, macaddr[4]);
	enc28j60Write(MAADR0, macaddr[5]);

	printf("MAADR5 = 0x%x\r\n", enc28j60Read(MAADR5));
	printf("MAADR4 = 0x%x\r\n", enc28j60Read(MAADR4));
	printf("MAADR3 = 0x%x\r\n", enc28j60Read(MAADR3));
	printf("MAADR2 = 0x%x\r\n", enc28j60Read(MAADR2));
	printf("MAADR1 = 0x%x\r\n", enc28j60Read(MAADR1));
	printf("MAADR0 = 0x%x\r\n", enc28j60Read(MAADR0));

	enc28j60PhyWrite(PHCON1, PHCON1_PDPXMD);


	// no loopback of transmitted frames
	enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);
	// switch to bank 0
	enc28j60SetBank(ECON1);
	// enable interrutps
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
	// enable packet reception
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}

// read the revision of the chip:
unsigned char enc28j60getrev(void)
{
	return(enc28j60Read(EREVID));
}

void enc28j60PacketSend(unsigned int len, unsigned char* packet)
{
	// Set the write pointer to start of transmit buffer area
	enc28j60Write(EWRPTL, TXSTART_INIT&0xFF);
	enc28j60Write(EWRPTH, TXSTART_INIT>>8);

	// Set the TXND pointer to correspond to the packet size given
	enc28j60Write(ETXNDL, (TXSTART_INIT+len)&0xFF);
	enc28j60Write(ETXNDH, (TXSTART_INIT+len)>>8);

	// write per-packet control byte (0x00 means use macon3 settings)
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);

	// copy the packet into the transmit buffer
	enc28j60WriteBuffer(len, packet);

	// send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);

    // Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
	if( (enc28j60Read(EIR) & EIR_TXERIF) )
	{
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS);
    }
}

// Gets a packet from the network receive buffer, if one is available.
// The packet will by headed by an ethernet header.
//      maxlen  The maximum acceptable length of a retrieved packet.
//      packet  Pointer where packet data should be stored.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
unsigned int enc28j60PacketReceive(unsigned int maxlen, unsigned char* packet)
{
	unsigned int rxstat;
	unsigned int len;

	// check if a packet has been received and buffered
	//if( !(enc28j60Read(EIR) & EIR_PKTIF) ){
        // The above does not work. See Rev. B4 Silicon Errata point 6.
	if( enc28j60Read(EPKTCNT) ==0 )
	{
		return(0);
    }
	// Set the read pointer to the start of the received packet
	enc28j60Write(ERDPTL, (NextPacketPtr));
	enc28j60Write(ERDPTH, (NextPacketPtr)>>8);

	// read the next packet pointer
	NextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	NextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;

	// read the packet length (see datasheet page 43)
	len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
        // read the receive status (see datasheet page 43)
    rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
    rxstat |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
    // limit retrieve length
    if (len>maxlen-1)
    {
        len=maxlen-1;
    }
     // check CRC and symbol errors (see datasheet page 44, table 7-3):
    // The ERXFCON.CRCEN is set by default. Normally we should not
    // need to check this.
    if ((rxstat & 0x80)==0)
    {
        // invalid
        len=0;
    }
    else
    {
        // copy the packet from the receive buffer
        enc28j60ReadBuffer(len, packet);
    }

	// Move the RX read pointer to the start of the next received packet
	// This frees the memory we just read out
	enc28j60Write(ERXRDPTL, (NextPacketPtr));
	enc28j60Write(ERXRDPTH, (NextPacketPtr)>>8);
	// decrement the packet counter indicate we are done with this packet
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
	return(len);
}



