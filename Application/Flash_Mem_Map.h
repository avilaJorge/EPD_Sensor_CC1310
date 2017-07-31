/**
* Copyright (c) Pervasive Displays Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. The name of Atmel may not be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an
*    Atmel microcontroller product.
*
* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef SRC_APP_FLASH_MEM_MAP_H_
#define SRC_APP_FLASH_MEM_MAP_H_
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#define TemporaryMinSize		50
#define TemporaryMaxSize		62
#define BlockMaxSize  			62
#define FunctionBlock  			63

#define PayloadLen 				32
#define BlockSector 			4
#define ImageInfoSize 			32
#define SectorSize 				(1024 * BlockSector)
#define BlockSize 				(uint32_t)(BlockSector * SectorSize)//1 Block=16 KBytes

#define BootBlockMax			32
#define FunBootAddress  		(uint32_t)(FunctionBlock * BlockSize)

#define RunBlockMax				50
#define FunRunAddress  			(uint32_t)((FunctionBlock * BlockSize) + SectorSize)

uint32_t GetFunBootAddress(int index);
uint32_t GetFunRunAddress(int index);
uint32_t GetBlockAddress(int BlockIndex);
uint32_t GetImageInfoAddress(int BlockIndex);
uint32_t GetTemporaryAddress(int index);

#endif /* SRC_APP_FLASH_MEM_MAP_H_ */
