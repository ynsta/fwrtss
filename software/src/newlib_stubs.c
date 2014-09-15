/**
 * @file newlib_stubs.c
 * @brief Newlib basic implementation
 * @author Stany MARCEL
 * @date 2014
 */

/*
 *  Copyright (c) 2014, Stany MARCEL <stanypub@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 *  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 *  DAMAGE.
 */

#include <sys/stat.h>
#include <errno.h>
#include <sys/times.h>

#include "stm32f302x8.h"
#include "stm32f3xx_hal.h"
#include "uart_dma.h"

#undef errno

extern int errno;


char * __env[1] = { 0 };

/**
 * @brief A pointer to a list of environment variables and their values.
 */
char ** environ = __env;


/**
 * @brief Exit a program without cleaning up files.
 */
void _exit(int status)
{
    while (1)
        /* TODO: reset */;
}


/**
 * @brief Close a file.
 */
int _close(int file)
{
    return -1;
}

/**
 * @brief Transfer control to a new process. Minimal implementation (for a
 * system without processes).
 */
int _execve(char *name, char **argv, char **env){
    errno = ENOMEM;
    return -1;
}

/**
 * @brief Create a new process. Minimal implementation (for a system without
 * processes).
 */

int _fork() {
    errno = EAGAIN;
    return -1;
}

/**
 * @brief Return the status of an open file.
 */
int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}


/**
 * @brief Return Process-ID. Minimal implementation, for a system without
 * processes.
 */
int _getpid() {
    return 1;
}


/**
 * @brief Query whether output stream is a terminal.
 */
int _isatty(int file)
{
    return 1;
}

/**
 * @brief Send a signal.
 *
 * Minimal implementation:
 */
int _kill(int pid, int sig){
    errno = EINVAL;
    return(-1);
}

/**
 * @brief Establish a new name for an existing file. Minimal implementation.
 */
int _link(char *old, char *new){
    errno = EMLINK;
    return -1;
}

/**
 * @brief Set position in a file.
 */
int _lseek(int file, int ptr, int dir)
{
    return 0;
}

/**
 * @brief Open a file. Minimal implementation.
 */
int _open(const char *name, int flags, int mode)
{
    return -1;
}

/**
 * @brief Read from a file.
 */
int _read(int file, char *ptr, int len)
{
    if (file != 1 && file != 2)
        return 0;

    return uart_dma_read(ptr, len);
}

/**
 * @brief Increase program data space.
 */
caddr_t _sbrk(int incr)
{
    static uint32_t heap_end = 0;
    uint32_t heap_end_prev;
    uint32_t sp = __get_MSP();

    extern void * _sstack; /* from linker */

    if (0 == heap_end)
        heap_end = (uint32_t)&_sstack;

    if (heap_end + incr > sp) {
        return (caddr_t) 0xFFFFFFFF;
    }

    heap_end_prev = heap_end;
    heap_end     += incr;

    return (caddr_t) heap_end_prev;
}

/**
 * @brief Status of a file (by name). Minimal implementation.
 */
int _stat(const char *file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

/**
 * @brief Timing information for current process. Minimal implementation.
 */
clock_t _times(struct tms *buf)
{
    return -1;
}

/**
 * @brief Remove a file's directory entry. Minimal implementation.
 */
int _unlink(char *name)
{
    errno = ENOENT;
    return -1;
}

/**
 * @brief Wait for a child process. Minimal implementation.
 */
int _wait(int *status)
{
    errno = ECHILD;
    return -1;
}

/**
 * @brief Write to a file.
 */
int _write(int file, char *ptr, int len)
{
    if (file != 1 && file != 2)
        return -1;

    return uart_dma_write(ptr, len);
}


void __attribute__((weak)) _init(void)
{
    return ;
}

void __attribute__((weak)) _fini(void)
{
    return ;
}
