#pragma once

#pragma pack(push, 1)
struct CmdMaster
{
	uint8_t cmd;
	uint8_t data1;
	int16_t data2;
};
#pragma pack(pop)

struct CmdSlave
{
	float data1;
	float data2;
	float data3;
	float data4;
	float data5;
	float data6;
};

