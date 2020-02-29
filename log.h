/*
 * =====================================================================================
 *
 *       Filename:  log.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2018年05月23日 11时42分39秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  fenglilei (), lilei.feng@pku.edu.cn
 *        Company:  Peking University
 *
 * =====================================================================================
 */

#ifndef __LOG_H__
#define __LOG_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum level{
	FATAL = 0,
	ERR,
	CRIT,
	INFO,
	DEBUG
};

typedef struct{
	enum level log_level;
	char log_str[10];
}log_ele_t;

static log_ele_t log_tbl[]={
	{ FATAL, "FATAL"},
	{ ERR, "ERR"},
	{ CRIT, "CRIT"},
	{ INFO, "INFO"},
	{ DEBUG, "DEBUG"}
};


#define LOG_LEVEL INFO

#define LOG(level, format, ...)                                                                           \
do{                                                                                                    \
    time_t t = time(0);                                                                                 \
    struct tm ttt = *localtime(&t);                                                                     \
    if(level<=LOG_LEVEL){\
    fprintf(stdout, "[%s] [%4d-%02d-%02d %02d:%02d:%02d] [%s:%d] " format "\n",                     \
            log_tbl[level].log_str, ttt.tm_year + 1900, ttt.tm_mon + 1, ttt.tm_mday, ttt.tm_hour,        \
            ttt.tm_min, ttt.tm_sec, __FUNCTION__ , __LINE__, ##__VA_ARGS__);                            \
	}\
}while(0)


#endif		//__LOG_H__
