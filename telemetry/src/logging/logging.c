#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "logging.h"
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ctype.h>

#include <dirent.h>
#include <time.h>

/* The maximum size of a log file's file name. */

#define MAX_FILENAME 32

/* The format for flight log file names */

#define FLIGHT_FNAME_FMT CONFIG_INSPACE_TELEMETRY_FLIGHT_FS "/flog_boot%d_%d.bin"

/* The format for extraction log file names */

#define EXTR_FNAME_FMT CONFIG_INSPACE_TELEMETRY_LANDED_FS "/elog_boot%d_%d.bin"

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

/* The number of times opening a log file will be attempted */

#define NUM_TIMES_TRY_OPEN 3


int find_max_boot_number(char *file){
  DIR *directory_pointer = opendir("./");
  struct dirent *entry;

  int max_boot_number = 0;

  if (directory_pointer != NULL)
  {
    while ((entry = readdir(directory_pointer)) != NULL)
    {
      if (strstr(entry->d_name, file) != NULL) 
      {
        // Ex.) find /log/elog in /log/elog2. Index should be 8 to get number 2
        int index = strlen(file);
        char char_num = entry->d_name[index];
        if (!isdigit(char_num))
        {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
          fprintf(stderr, "Number after logging file does not exist (%s)\n", file);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
          return -1;
        }

        int boot_number = char_num - '0'; // Convert from character to int using ASCII values
        if (boot_number > max_boot_number)
        {
          max_boot_number = boot_number;
        }
      }
    }

    (void)closedir(directory_pointer);
    return max_boot_number;
  }
  else
  {
    perror("Couldn't open the directory: "); 
    return -1;
  }
}

/*
 * Returns double time difference between two timespec structs
 */
double timespec_diff(struct timespec *start, struct timespec *end)
{
  return (end->tv_sec - start->tv_sec) + (end->tv_nsec - start->tv_nsec) / 1e9;
}

/*
 * Logging thread which runs to log data to the SD card.
 */
void *logging_main(void *arg)
{
  int err;
  size_t written;
  enum flight_state_e flight_state;
  rocket_state_t *state = ((rocket_state_t *)(arg));
  uint32_t version = 0;
  char flight_filename1[sizeof(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS) + MAX_FILENAME];
  char flight_filename2[sizeof(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS) + MAX_FILENAME];
  char land_filename[sizeof(CONFIG_INSPACE_TELEMETRY_LANDED_FS) + MAX_FILENAME];

  FILE *storage_file_1 = NULL;    // File pointer for first logging file
  FILE *storage_file_2 = NULL;    // File pointer for second logging file
  FILE *active_storage_file;      // File pointer for active logging file

  FILE *extract_storage_file;      // File pointer for extraction logging file

  struct stat file_info;          // Stat struct to hold fstat return
  struct timespec base_timespec;  // Time that file was last modified
  struct timespec new_timespec;   // Time to check against base_time to see if 30 seconds has passed

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  printf("Logging thread started.\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

  /* Generate flight log file names using the boot number */

  int max_flight_log_boot_number = find_max_boot_number(CONFIG_INSPACE_TELEMETRY_FLIGHT_FS "/elog_boot");
  if (max_flight_log_boot_number < 0)
  {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Error finding max boot number: return (%d)\n", max_flight_log_boot_number);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }

  snprintf(flight_filename1, sizeof(flight_filename1), FLIGHT_FNAME_FMT, max_flight_log_boot_number, 1);
  snprintf(flight_filename2, sizeof(flight_filename1), FLIGHT_FNAME_FMT, max_flight_log_boot_number, 2);

  if (strlen(flight_filename1) > MAX_FILENAME) // Check log filename length
  {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Log file's name (%s) is longer than the maximum size of characters (%d)\n", flight_filename1, MAX_FILENAME);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }
  if (strlen(flight_filename2) > MAX_FILENAME) // Check log filename length
  {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Log file's name (%s) is longer than the maximum size of characters (%d)\n", flight_filename2, MAX_FILENAME);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }

  for (int i = 0; i < NUM_TIMES_TRY_OPEN; i++) // Matteo recomendeded trying multiple times, in case singular one fails.
  {
    storage_file_1 = fopen(flight_filename1, "rb+"); // Create file for reading and writing
    if (storage_file_1 == NULL)
    {
      err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      fprintf(stderr, "Error opening log file 1 (Attempt %d)'%s': %d\n", i, flight_filename1, err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

      //TODO: check errno values (if needed)
    }

    usleep(1 * 1000); // Sleep for 1 millisecond before trying again
  }

  if (storage_file_1 == NULL)
  {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Error opening log file 1 (%s): %d\n", flight_filename1, err);
#endif                            /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    pthred_exit(err_to_ptr(err)); // TODO: fail more gracefully
  }

  active_storage_file = storage_file_1;

  err = fstat(fileno(active_storage_file), &file_info);
  if (err)
  {
    err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Error using fstat: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    fclose(active_storage_file);
  }

  //Save the time the file was last modified
  base_timespec = (struct timespec){file_info.st_mtime, 0};
  // base_timespec.tv_sec = file_info.st_mtime;
  // base_timespec.tv_nsec = 0;

  /* Infinite loop to handle states */
  for (;;)
  {
    err = state_get_flightstate(state, &flight_state);
    if (err)
    {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      fprintf(stderr, "Error getting flight state: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      fclose(active_storage_file);
      // TODO: figure out fail conditions
    }

//     err = fstat(fileno(active_storage_file), &file_info);
//     if (err)
//     {
//       err = errno;
// #if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
//       fprintf(stderr, "Error using fstat: %d\n", err);
// #endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
//       fclose(active_storage_file);
//     }

    switch (flight_state)
    {
    case STATE_IDLE:
      /* Purposeful fall-through */
      
      // Switch between files every 30 seconds
      

      /* Store current system time as new_timespec */
      if (clock_gettime(CLOCK_REALTIME, new_timespec) < 0) // TODO: Is CLOCK_REALTIME the correct option?
      {
        err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during clock_gettime: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      double time_diff = timespec_diff(&base_timespec, &new_timespec);
      if (time_diff < 0)
      {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during timespec_diff (Negative difference returned): %f\n", time_diff);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      if (time_diff >= 30.0)
      {
        if (active_storage_file == storage_file_1)
        {
          /* Create new log file */
          for (int i = 0; i < NUM_TIMES_TRY_OPEN; i++) // Matteo recomendeded trying multiple times, in case singular one fails.
          {
            // storage_file_2 = open(flight_filename2, O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH); // Create file
            storage_file_2 = fopen(flight_filename2, "rb+"); // Create file
            if (storage_file_2 == NULL)
            {
              err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
              fprintf(stderr, "Error opening log file 2 '%s': %d\n", flight_filename2, err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

              // TODO: check errno values
              usleep(1 * 1000); // Sleep for 1 millisecond before trying again
              continue;
            }

            break;
          }

          if (storage_file_2 == NULL)
          {
            err = errno;
            pthread_exit(err_to_ptr(err)); // TODO: fail more gracefully
          }

          active_storage_file = storage_file_2;

          /* Store current system time as the new base time */
          if (clock_gettime(CLOCK_REALTIME, base_timespec) < 0) // TODO: Is CLOCK_REALTIME the correct option?
          {
            err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
            fprintf(stderr, "Error during clock_gettime: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
          }
        }
        else if (active_storage_file == storage_file_2)
        {
          active_storage_file = storage_file_1;
        }
      }

    case STATE_AIRBORNE:
    {
      /* Infinite loop to log data */

      /* Wait for the data to have a change */

      err = state_wait_for_change(state, &version);
      if (err)
      {
        err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during state_wait_for_change: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      /* Log data */

      // Sample logging
      uint8_t sample_buf[1] = {1};
      written = fwrite(sample_buf, 1, 1, active_storage_file);

      /* Store current system time as new_timespec */
      if (clock_gettime(CLOCK_REALTIME, new_timespec) < 0) // TODO: Is CLOCK_REALTIME the correct option?
      {
        err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during clock_gettime: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      double diff = timespec_diff(&base_timespec, &new_timespec);
      if (diff < 0)
      {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during timespec_diff (Negative difference returned): %f\n", diff);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }
      

      if (diff >= 30.0)
      {
        /* Create new log file */
        for (int i = 0; i < NUM_TIMES_TRY_OPEN; i++) // Matteo recomendeded trying multiple times, in case singular one fails.
        {
          // storage_file_2 = open(flight_filename2, O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH); // Create file
          storage_file_2 = fopen(flight_filename2, "rb+"); // Create file
          if (storage_file_2 == NULL)
          {
            err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
            fprintf(stderr, "Error opening log file 2 '%s': %d\n", flight_filename2, err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

            // TODO: check errno values

            usleep(1 * 1000); // Sleep for 1 millisecond before trying again
            continue;
          }

          break;
        }

        if (storage_file_2 == NULL)
        {
          err = errno;
          pthread_exit(err_to_ptr(err)); // TODO: fail more gracefully
        }

        active_storage_file = storage_file_2;

        /* Store current system time as the new base time */
        if (clock_gettime(CLOCK_REALTIME, base_timespec) < 0) // TODO: Is CLOCK_REALTIME the correct option?
        {
          err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
          fprintf(stderr, "Error during clock_gettime: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
        }
      }

      err = state_read_lock(state);

      if (err)
      {
        err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during state_read_lock: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      written = fwrite(&state->data, sizeof(state->data), 1, active_storage_file);
      // written = write(active_storage_file, sizeof(state->data), 1);
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      printf("Logged %u bytes\n", written * sizeof(state->data));
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      if (written == 0)
      { 
        // TODO: Handle error (might happen if file got too large, start
        // another file)
      }

      err = state_unlock(state);
      if (err)
      {
        err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error releasing read lock: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      /* If we are in the idle state, only write the latest n seconds of data*/
      if (flight_state == STATE_IDLE)
      {
        // TODO: check file position
      }
    }
    break;

    case STATE_LANDED:
    {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      printf("Copying files to extraction file system.\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

      /* Generate log file name for extraction file system */

      int max_extraction_log_file_number = find_max_boot_number(CONFIG_INSPACE_TELEMETRY_LANDED_FS "/flog_boot");

      snprintf(land_filename, sizeof(land_filename), EXTR_FNAME_FMT, max_extraction_log_file_number);

      /* Open extraction log file */

      for (int i = 0; i < NUM_TIMES_TRY_OPEN; i++) // Matteo recomendeded trying multiple times, in case singular one fails.
      {
        extract_storage_file = fopen(land_filename, "wb");
        if (extract_storage_file == NULL)
        {
          err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
          fprintf(stderr, "Error (Attempt %d) opening the extraction log file '%s': %d\n", i, land_filename, err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

          // TODO: check errno values
          usleep(1 * 1000); // Sleep for 1 millisecond before trying again
        }
        else
        {
          break;
        }        
      }

      if (extract_storage_file == NULL)
      {
        // If reach here, all attempts to open log file have failed, fatal error

        err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Couldn't open extraction log file '%s' with error: %d\n", land_filename, err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
        pthread_exit(err_to_ptr(err));
      }
      
      /* Roll power-safe log file pointer back to beginning */
      err = fseek(active_storage_file, 0, SEEK_SET); // TODO: handle error
      if (err)
      {
        /* Handle Error */
      }
      
      /* Copy from one to the other using a buffer */
      uint8_t buf[BUFSIZ];
      size_t nbytes = 0;

      while (!feof(active_storage_file))
      {
        nbytes = sizeof(buf) * fread(buf, sizeof(buf), 1, active_storage_file);
        if (nbytes == 0)
          break;
        nbytes = sizeof(buf) * fwrite(buf, nbytes, 1, extract_storage_file);
      }

      /* Now that logs are copied to FAT partition, move back to the idle state
       * for another go.
       * flight_state = STATE_IDLE;
       */

      if (fclose(extract_storage_file) != 0)
      {
          err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Couldn't close extraction log file '%s' with error: %d\n", land_filename, err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      err = state_set_flightstate(state, STATE_IDLE); // TODO: error handling
    }
    }
  }

#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
  if (close(active_storage_file) != 0)
  {
    err = errno;
    fprintf(stderr, "Failed to close flight logfile handle: %d\n", err);
  }
#else
  close(active_storage_file);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  pthread_exit(err_to_ptr(err));


/* Teardown */
  teardown(storage_file_1, storage_file_2, extract_storage_file)

}

void teardown(FILE *flight_log_1, FILE* flight_log_2, FILE* extraction_file_ptr)
{
  int err;
  if (fclose(flight_log_1) != 0)
  {
    err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Failed to close flight logfile 1 handle: %d\n", err);
#endif
  }

  if (fclose(flight_log_2) != 0)
  {
    err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Failed to close flight logfile 2 handle: %d\n", err);
#endif
  }

  if (fclose(extraction_file_ptr) != 0)
  {
    err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Failed to close extraction logfile handle: %d\n", err);
#endif
  }
}
