/**********************************************************************
** Copyright 2008-2013 OPPO Mobile Comm Corp., Ltd, All rights reserved.
** VENDOR_EDIT :                                                                                             
** File : - SDM660_8.0_LA_2.0\android\kernel\msm-4.4\include\soc\device_info.h
** ModuleName:devinfo
** Author : wangjc
** Version : 1.0
** Date : 2013-10-23
** Descriptio : add interface to get device information.
** History :
**                  <time>         <author>             <desc>
**                2013-10-23	    wangjc	            init
**********************************************************************/

#ifndef _DEVICE_INFO_H
#define _DEVICE_INFO_H


/*dram type*/
/*
enum{
        DRAM_TYPE0 = 0,
        DRAM_TYPE1,
        DRAM_TYPE2,
        DRAM_TYPE3,
        DRAM_UNKNOWN,
};
*/

enum{
        MAINBOARD_RESOURCE0 = 0,
        MAINBOARD_RESOURCE1 = 1,
        MAINBOARD_RESOURCE2 = 2,
};

enum {
        OPPO_17107 = 17107,
        OPPO_17108 = 17108,
        OPPO_17109 = 17109,
        OPPO_17127 = 17127,
        OPPO_17128 = 17128,
        OPPO_18181 = 18181,
};

enum{
        OPERATOR_OPEN_MARKET          = 1,
        OPERATOR_CHINA_MOBILE             = 2,
        OPERATOR_FOREIGN                             = 5,
/*#ifdef VENDOR_EDIT*/
        OPERATOR_FOREIGN_WCDMA         = 6,   /*qifeng.liu 2014.08.07 FOR MAC*/
        OPERATOR_FOREIGN_RESERVED   = 7,    /*shaoming 2014/10/04 add for 14085's dual sim version*/
        OPERATOR_ALL_CHINA_CARRIER    = 8,
        OPERATOR_ALL_CHINA_CARRIER_MOBILE = 9,
        OPERATOR_ALL_CHINA_CARRIER_UNICOM = 10,
};

struct manufacture_info {
        char *version;
        char *manufacture;
        char *fw_path;
};

int register_device_proc(char *name, char *version, char *manufacture);
int register_devinfo(char *name, struct manufacture_info *info);


#endif /*_DEVICE_INFO_H*/
