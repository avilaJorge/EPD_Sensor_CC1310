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

#include <HW_drivers/Flash_Mem_Map.h>
#include "conf_EPD.h"
#include "Data_Module.h"
uint32_t GetFunBootAddress(int index)
{
	if(index>BootBlockMax) index=BootBlockMax-1;
	return (uint32_t)(FunBootAddress + (uint32_t)((sizeof(Boot_Info_t)-sizeof(Flash_Parameter_Info_t))*index));
}
uint32_t GetFunRunAddress(int index)
{
	if(index>RunBlockMax) index=RunBlockMax-1;
	return (uint32_t)(FunRunAddress + (uint32_t)((sizeof(RunImage_Info_t)-sizeof(Flash_Parameter_Info_t))*index));
}
uint32_t GetBlockAddress(int BlockIndex)
{
	if (BlockIndex > BlockMaxSize) BlockIndex = BlockMaxSize;
	return (uint32_t)(BlockSize * BlockIndex);
}
uint32_t GetImageInfoAddress(int BlockIndex)
{
	if (BlockIndex > BlockMaxSize) BlockIndex = BlockMaxSize;
	return (uint32_t)((BlockSize * (BlockIndex + 1)) - ImageInfoSize);
}
uint32_t GetTemporaryAddress(int index)
{
	return  (uint32_t)(index * BlockSize);
}
