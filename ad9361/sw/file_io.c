/***************************************************************************//**
 *   @file   file_io.h
 *   @brief  Header file of file io module
 *   @author Colden Rouleau (colden.rouleau@colorado.edu)
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "stdint.h"
#include <string.h>
#include "file_io.h"

FILE *hw_interface_file;

/***************************************************************************//**
 * @brief spi_log_open -- opens a text file for logging SPI read/write
*******************************************************************************/
void spi_log_open(const char *filename)
{
    hw_interface_file = fopen(filename, "w");
}

/***************************************************************************//**
 * @brief spi_log_close -- closes the file handle for logging SPI read/write
*******************************************************************************/
void spi_log_close(void)
{
    fclose(hw_interface_file);
}

void log_string(const char *str)
{
    fputs(str,hw_interface_file);
}

