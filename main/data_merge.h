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
 * Skips the header row of the incoming file and any rows whose device_mac
 * field matches @p skip_mac (pass NULL to disable this filter).
 * Per-MAC timestamp deduplication prevents re-appending records already
 * present in CSV_MERGED_FILE.
 *
 * @param incoming_path  Full path to the received CSV temp file on the SD card.
 * @param skip_mac       4-char device MAC string to exclude (e.g. own MAC to
 *                       prevent echo-back), or NULL to include all rows.
 * @return Number of records appended, or -1 on error.
 */
int data_merge_from_file(const char *incoming_path, const char *skip_mac);
