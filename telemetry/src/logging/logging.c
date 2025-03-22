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

// Private Functions 

static int find_max_boot_number(char *file);
static int switch_active_log_file(FILE **active_storage_file, FILE *storage_file_1, FILE *storage_file_2, char *flight_filename2);
static double timespec_diff(struct timespec *new_time, struct timespec *old_time);
static int try_open_file(FILE **file_to_open, char *filename, char *open_option);

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

  FILE *storage_file_1 = NULL;        // File pointer for first logging file
  FILE *storage_file_2 = NULL;        // File pointer for second logging file
  FILE *active_storage_file = NULL;   // File pointer for active logging file

  FILE *extract_storage_file;         // File pointer for extraction logging file

  struct stat file_info;              // Stat struct to hold fstat return
  struct timespec base_timespec;      // Time that file was last modified
  struct timespec new_timespec;       // Time to check against base_time to see if 30 seconds has passed

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

  snprintf(flight_filename1, sizeof(flight_filename1), FLIGHT_FNAME_FMT, max_flight_log_boot_number + 1, 1);
  snprintf(flight_filename2, sizeof(flight_filename1), FLIGHT_FNAME_FMT, max_flight_log_boot_number + 1, 2);

  if (strlen(flight_filename1) > MAX_FILENAME) // Check log 1 filename length
  {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Log file's name (%s) is longer than the maximum size of characters (%d)\n", flight_filename1, MAX_FILENAME);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }
  if (strlen(flight_filename2) > MAX_FILENAME) // Check log 2 filename length
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

  int fd = fileno(active_storage_file);
  if (fd < 0)
  {
    err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Error using fileno: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    fclose(active_storage_file);
  }

  err = fstat(fd, &file_info);
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

    switch (flight_state)
    {
    case STATE_IDLE:
      /* Purposeful fall-through */
      
      // Switch between files every 30 seconds

      err = state_wait_for_change(state, &version);
      if (err)
      {
        err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during state_wait_for_change: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      /* Store current system time as new_timespec */
      if (clock_gettime(CLOCK_REALTIME, &new_timespec) < 0) // TODO: Is CLOCK_REALTIME the correct option?
      {
        err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during clock_gettime: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      double time_diff = timespec_diff(&new_timespec, &base_timespec);
      if (time_diff < 0)
      {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during timespec_diff (Negative difference returned): %f\n", time_diff);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      if (time_diff >= 30.0)
      {
        // Switch active log file
        err = switch_active_log_file(&active_storage_file, &storage_file_1, &storage_file_2, &flight_filename2);
        if (err < 0)
        {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
          fprintf(stderr, "Error switching active log file: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
          err = errno;
          pthread_exit(err_to_ptr(err)); // TODO: fail more gracefully
        }

//         /* Create new log file */
//         for (int i = 0; i < NUM_TIMES_TRY_OPEN; i++) // Matteo recomendeded trying multiple times, in case singular one fails.
//         {
//           // storage_file_2 = open(flight_filename2, O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH); // Create file
//           storage_file_2 = fopen(flight_filename2, "rb+"); // Create file
//           if (storage_file_2 == NULL)
//           {
//             err = errno;
// #if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
//             fprintf(stderr, "Error opening log file 2 '%s': %d\n", flight_filename2, err);
// #endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
//
//             // TODO: check errno values
//
//             usleep(1 * 1000); // Sleep for 1 millisecond before trying again
//             continue;
//           }
//
//           break;
//         }
//
//         if (storage_file_2 == NULL)
//         {
//           err = errno;
//           pthread_exit(err_to_ptr(err)); // TODO: fail more gracefully
//         }
//
//         active_storage_file = storage_file_2;

        /* Set Base time to current system time */
        if (clock_gettime(CLOCK_REALTIME, &base_timespec) < 0) // TODO: Is CLOCK_REALTIME the correct option?
        {
          err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
          fprintf(stderr, "Error during clock_gettime: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
        }
      }

      err = state_read_lock(state); // Should this be before ???
      if (err)
      {
        err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during state_read_lock: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      written = fwrite(&state->data, sizeof(state->data), 1, active_storage_file);
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
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during state_unlock: %d\n", err);
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
      int max_extraction_log_file_number = find_max_boot_number(CONFIG_INSPACE_TELEMETRY_LANDED_FS "/elog_boot");
      snprintf(land_filename, sizeof(land_filename), EXTR_FNAME_FMT, max_extraction_log_file_number, 1);

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
      if (err < 0)
      {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during state_set_flightstate: %d\n", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }
      
    }
    }
  }

  /* Close files that may be open */
  if (fclose(storage_file_1) != 0)
  {
    err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Failed to close flight logfile 1 handle: %d\n", err);
#endif
  }

  if (fclose(storage_file_2) != 0)
  {
    err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Failed to close flight logfile 2 handle: %d\n", err);
#endif
  }

  pthread_exit(err_to_ptr(err));
}



/****************************************************/
/*                Helper Functions                  */
/****************************************************/

/**
 * Attempt to open file given filename and flag option multiple times. Once opened, update active file pointer.
 *
 * @param active_file_pointer The pointer to the file to update
 * @param filename The file name of the file to open
 * @param open_option The string option passed to fopen (r, rb, rb+, etc)
 * @return 0 if succesful, else error from errno
 */
static int try_open_file(FILE** active_file_pointer, char* filename, char* open_option){
  int err = 0;
  for (int i = 0; i < NUM_TIMES_TRY_OPEN; i++) // Matteo recomendeded trying multiple times, in case singular one fails.
  {
    *active_file_pointer = fopen(filename, open_option);
    if (*active_file_pointer == NULL)
    {
      err = errno;
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      fprintf(stderr, "Error (Attempt %d) opening '%s' file: %d\n", i, filename, err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */

      // TODO: check errno values
      usleep(1 * 1000); // Sleep for 1 millisecond before trying again
    }
    else
    {
      break;
    }
  }

  return err;
}

/**
 * Find max boot number of logging files that already exist
 *
 * @param filename The format string name that will be checked
 * @return The maximum boot number found of previous files, -1 if error 
 **/
static int find_max_boot_number(char *filename)
{
  int max_boot_number = 0;

  DIR *directory_pointer = opendir("./");
  if (directory_pointer == NULL)
  {
    perror("Couldn't open the directory: ");
    return -1;
  }
  
  struct dirent *entry;
  while ((entry = readdir(directory_pointer)) != NULL)
  {
    if (strstr(entry->d_name, filename) != NULL)
    {
      int boot_number, file_number;
      int vars_filled = sscanf(filename, FLIGHT_FNAME_FMT, &boot_number, &file_number);

      if (vars_filled != 2)
      {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
        fprintf(stderr, "Error during sscanf: 2 values not filled");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      }

      if (boot_number > max_boot_number)
      {
        max_boot_number = boot_number;
      }
    }
  }

  (void)closedir(directory_pointer);
  return max_boot_number;
}

/**
 * Switch the active file pointer for the ping pong buffer
 *
 * @param active_storage_file The active log file pointer
 * @param storage_file_1 The pointer to the first log file
 * @param storage_file_2 The pointer to the second log file
 * @param flight_filename2 The name of the second log file
 * @return 0 if succesful, -1 on error
 **/
static int switch_active_log_file(FILE **active_storage_file, FILE *storage_file_1, FILE *storage_file_2, char *flight_filename2)
{
  int err = 0; 

  /* Make sure we aren't dereferencing null pointer later */
  if (*active_storage_file == NULL)
  {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Error in switch_active_log_file: active_storage_file is NULL");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    return -1;
  }
  
  if (*active_storage_file == storage_file_1)
  {
    if (storage_file_2 != NULL){
      *active_storage_file = storage_file_2;
      return err;
    }

    /* If reach here, file 2 is not opened yet, so open it*/
    err = try_open_file(&storage_file_2, flight_filename2, "rb+");
    if (err < 0 || storage_file_2 == NULL)
    {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      fprintf(stderr, "Error (%d) trying to open %s", err, flight_filename2);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      return -1;
    }

    *active_storage_file = storage_file_2;
  }

  // If file_2 is currently active, that means file_1 was opened successfully, so just switch the pointer
  else if (*active_storage_file == storage_file_2)
  {
    if (storage_file_1 == NULL)
    {
      return -1;
    }

    *active_storage_file = storage_file_1;
  }

  /* Roll active log file pointer back to beginning */
  err = fseek(*active_storage_file, 0, SEEK_SET);
  if (err)
  {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Couldn't seek active file back to start: %d", err);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
  }

  return err;
}


/**
 * Returns double time difference between two timespec structs
 *
 * @param new_time The newer (larger) time 
 * @param old_time The older (smaller) time
 * @return The difference between the two times
 **/
static double timespec_diff(struct timespec *new_time, struct timespec *old_time)
{
  return (new_time->tv_sec - old_time->tv_sec) + (new_time->tv_nsec - old_time->tv_nsec) / 1e9;
}