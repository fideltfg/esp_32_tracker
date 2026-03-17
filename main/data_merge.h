#pragma once
#include <stdbool.h>
#include <stddef.h>

/**
 * @file data_merge.h
 * @brief Append CSV records received from peers into CSV_MERGED_FILE.
 *
 * The server is responsible for deduplication across data.csv and merged.csv.
 * On the device we simply collect all incoming records without checking for
 * duplicates, which eliminates the in-RAM keyset and the SD read pass.
 */

/**
 * @brief Append all data rows from an incoming CSV file to CSV_MERGED_FILE.
 *
 * Skips the header row of the incoming file. No deduplication is performed.
 *
 * @param incoming_path  Full path to the received CSV temp file on the SD card.
 * @return Number of records appended, or -1 on error.
 */
int data_merge_from_file(const char *incoming_path);
