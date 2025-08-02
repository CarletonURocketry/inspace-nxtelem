#include <ftw.h>
#include <limits.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <sys/stat.h>
#include <testing/unity.h>

#include "../telemetry/src/logging/logging.h"

#define TEST_DIR CONFIG_INSPACE_TELEMETRY_FLIGHT_FS "testing"
#define TEST_FLIGHT_FS_DIR CONFIG_INSPACE_TELEMETRY_FLIGHT_FS "fs_test"
#define TEST_LANDED_FS_DIR CONFIG_INSPACE_TELEMETRY_LANDED_FS "fs_test"

#define TEST_SWAP_DIR TEST_DIR "/test_swap"
#define TEST_FLIGHT_NUM_DIR TEST_DIR "/test_flight_num"
#define TEST_OPEN_DIR TEST_DIR "/test_open"
#define TEST_COPY_DIR TEST_DIR "/test_copy"

static void create_test_dir(const char *test_dir) {
    if (mkdir(test_dir, S_IWUSR | S_IRUSR) < 0) {
        int err = errno;
        printf("Couldn't make the test directory %s: %d\n", test_dir, err);
    }
}

static int remove_path(const char *pathname, const struct stat *sbuf, int type, struct FTW *ftwb) {
    if (remove(pathname) < 0) {
        perror("Couldn't clean up a path");
        return -1;
    }
    return 0;
}

static void remove_test_dir(const char *test_dir) {
    if (nftw(test_dir, &remove_path, 10, FTW_DEPTH | FTW_MOUNT | FTW_PHYS) < 0) {
        perror("Couldn't clean up directory");
    }
}

static void test_filesystem(const char *test_dir) {
    create_test_dir(test_dir);

    char filename[PATH_MAX];
    snprintf(filename, sizeof(filename), "%s/test_file", test_dir);

    // Try open
    FILE *file_test = fopen(filename, "w");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(NULL, file_test, "Could not create a file");

    // Try write
    char data[] = "TEST DATA";
    TEST_ASSERT_EQUAL_MESSAGE(sizeof(data), fwrite(data, 1, sizeof(data), file_test),
                              "Could not write test data to a file");

    // Try close
    TEST_ASSERT_EQUAL_MESSAGE(0, fclose(file_test), "Could not to close a file");

    file_test = fopen(filename, "w+");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(NULL, file_test, "Could not reopen a file");

    // Try read
    char buffer[sizeof(data) + 1];
    int len = fread(buffer, 1, sizeof(buffer), file_test);
    TEST_ASSERT_EQUAL_MESSAGE(sizeof(data), len, "Did not read same data from file that was written to");

    // Try seek
    TEST_ASSERT_EQUAL_MESSAGE(0, fseek(file_test, 0, SEEK_SET), "Could not seek file to beginning");
    len = fread(buffer, 1, sizeof(buffer), file_test);
    TEST_ASSERT_EQUAL_MESSAGE(sizeof(data), len, "Did not read the same data after seeking");

    // Try truncate
    int fd = fileno(file_test);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, fd, "fileno could not get fd");
    TEST_ASSERT_EQUAL_MESSAGE(0, ftruncate(fd, 0), "Could not truncate file to 0 length");
    len = fread(buffer, 1, sizeof(buffer), file_test);
    TEST_ASSERT_EQUAL_MESSAGE(0, len, "Truncate did not delete contents of file");

    remove_test_dir(test_dir);
}

static void test_flight_filesystem(void) { test_filesystem(TEST_FLIGHT_FS_DIR); }

static void test_landed_filesystem(void) { test_filesystem(TEST_LANDED_FS_DIR); }

static void test_should_swap(void) {
    struct timespec first = {0};
    struct timespec second = {0};

    /* No difference */
    TEST_ASSERT_FALSE_MESSAGE(should_swap(&first, &second), "Should not try to swap when timespecs are equal");

    /* Negative difference*/
    first.tv_sec = 1;
    TEST_ASSERT_FALSE_MESSAGE(should_swap(&first, &second), "Should not try to swap files when time diff is negative");

    /* 16 minute difference */
    second.tv_sec = 1000;
    TEST_ASSERT_TRUE_MESSAGE(should_swap(&first, &second), "Should try to swap files when time difference is huge");
}

static void test_swap_files(void) {
    create_test_dir(TEST_SWAP_DIR);

    char data[] = "TEST DATA";
    FILE *file_one = fopen(TEST_SWAP_DIR "/file_one", "w+");
    TEST_ASSERT(file_one);
    fwrite(data, sizeof(data), 1, file_one);

    char other_data[] = "OTHER TEST DATA";
    FILE *file_two = fopen(TEST_SWAP_DIR "/file_two", "w+");
    TEST_ASSERT(file_two);
    fwrite(other_data, sizeof(other_data), 1, file_two);

    FILE *active_file = file_one;
    FILE *standby_file = file_two;
    TEST_ASSERT_EQUAL_MESSAGE(0, swap_files(&active_file, &standby_file), "Swapping should have been successful");

    TEST_ASSERT_EQUAL_PTR_MESSAGE(file_two, active_file, "Swapping files should involve swapping pointers");
    TEST_ASSERT_EQUAL_PTR_MESSAGE(file_one, standby_file, "Swapping files should involve swapping pointers");

    char buffer[sizeof(data) + 1];
    fseek(standby_file, 0, SEEK_SET);
    int len = fread(buffer, 1, sizeof(buffer), standby_file);
    TEST_ASSERT_EQUAL_MESSAGE(sizeof(data), len, "Should have read same number of bytes as test data");
    TEST_ASSERT_EQUAL_MEMORY(data, buffer, sizeof(data));

    fseek(active_file, 0, SEEK_SET);
    len = fread(buffer, 1, sizeof(buffer), active_file);
    TEST_ASSERT_EQUAL_MESSAGE(0, len, "File should have been cleared when set to active");

    remove_test_dir(TEST_SWAP_DIR);
}

static void test_choose_flight_number(void) {
    create_test_dir(TEST_FLIGHT_NUM_DIR);
    /* Not cleaning up files - use a subdirectory to keep things seperate */
    char test_format[] = "test_format_%d_%d";
    char create_format[] = TEST_FLIGHT_NUM_DIR "/test_format_%d_%d";
    char filename[PATH_MAX];

    /* Test no files */
    int flight_number = choose_flight_number(TEST_FLIGHT_NUM_DIR, test_format);
    TEST_ASSERT_EQUAL_MESSAGE(1, flight_number, "Should get 1 for flight number with no files");

    /* Test one file */
    snprintf(filename, sizeof(filename), create_format, 0, 0);
    FILE *file_one = fopen(filename, "w");
    TEST_ASSERT(file_one);
    fclose(file_one);
    flight_number = choose_flight_number(TEST_FLIGHT_NUM_DIR, test_format);
    TEST_ASSERT_EQUAL_MESSAGE(1, flight_number, "Should get 1 for flight number with one file with 0 flight number");

    /* Test two files with the same flight number */
    snprintf(filename, sizeof(filename), create_format, 0, 1000);
    file_one = fopen(filename, "w");
    TEST_ASSERT(file_one);
    fclose(file_one);
    flight_number = choose_flight_number(TEST_FLIGHT_NUM_DIR, test_format);
    TEST_ASSERT_EQUAL_MESSAGE(1, flight_number, "Should get 1 for flight number with two files with 0 flight number");

    /* Test a file with a high flight number */
    snprintf(filename, sizeof(filename), create_format, 1000, 0);
    file_one = fopen(filename, "w");
    TEST_ASSERT(file_one);
    fclose(file_one);
    flight_number = choose_flight_number(TEST_FLIGHT_NUM_DIR, test_format);
    TEST_ASSERT_EQUAL_MESSAGE(1001, flight_number, "Should get next flight number in directory");

    remove_test_dir(TEST_FLIGHT_NUM_DIR);
}

static void test_open_log_file(void) {
    create_test_dir(TEST_OPEN_DIR);

    FILE *file;
    char test_format[] = TEST_OPEN_DIR "/test_format_%d_%d";
    TEST_ASSERT_EQUAL(0, open_log_file(&file, test_format, 0, 0, "w"));
    TEST_ASSERT_EQUAL(0, fclose(file));

    TEST_ASSERT_EQUAL(0, open_log_file(&file, test_format, 0, 0, "w"));
    TEST_ASSERT_EQUAL(0, fclose(file));

    TEST_ASSERT_EQUAL(0, open_log_file(&file, test_format, 1000, 0, "w"));
    TEST_ASSERT_EQUAL(0, fclose(file));

    TEST_ASSERT_EQUAL(0, open_log_file(&file, test_format, 0, 1000, "a"));
    TEST_ASSERT_EQUAL(0, fclose(file));

    TEST_ASSERT_EQUAL(0, open_log_file(&file, test_format, 0, 1000, "r"));
    TEST_ASSERT_EQUAL(0, fclose(file));

    TEST_ASSERT_EQUAL(0, open_log_file(&file, test_format, 0, 1000, "w+"));
    TEST_ASSERT_EQUAL(0, fclose(file));

    char long_pathname[] = TEST_OPEN_DIR "/very_very_very_very_very_very_very_very_long_name";
    TEST_ASSERT_GREATER_THAN_MESSAGE(NAME_MAX, sizeof(long_pathname),
                                     "Test name is not long enough to test MAX_NAME limit!");
    TEST_ASSERT_EQUAL_MESSAGE(0, open_log_file(&file, long_pathname, 0, 1000, "w"),
                              "Should still be able to open file with name that's too long");
    TEST_ASSERT_EQUAL(0, fclose(file));

    remove_test_dir(TEST_OPEN_DIR);
}

static void test_copy_out(void) {
    create_test_dir(TEST_COPY_DIR);

    char data[] = "TEST DATA";
    FILE *log_file = fopen(TEST_COPY_DIR "/log_file", "w+");
    TEST_ASSERT(log_file);
    fwrite(data, sizeof(data), 1, log_file);

    FILE *extract_file = fopen(TEST_COPY_DIR "/extract_file", "w+");
    TEST_ASSERT(extract_file);

    TEST_ASSERT_EQUAL_MESSAGE(0, copy_out(log_file, extract_file), "Copy out should have been successful");

    char buffer[sizeof(data) * 2] = {0};
    fseek(extract_file, 0, SEEK_SET);
    int len = fread(buffer, 1, sizeof(buffer), extract_file);
    printf("Read %s\n", buffer);
    TEST_ASSERT_EQUAL_MESSAGE(sizeof(data), len, "Should have read same number of bytes as test data before copy");
    TEST_ASSERT_EQUAL_MEMORY(data, buffer, sizeof(data));

    fseek(log_file, 0, SEEK_SET);
    len = fread(buffer, 1, sizeof(buffer), log_file);
    TEST_ASSERT_EQUAL_MESSAGE(0, len, "File should have been cleared when copied out");

    remove_test_dir(TEST_COPY_DIR);
}

void test_logging(void) {
    create_test_dir(TEST_DIR);

    RUN_TEST(test_flight_filesystem);
    RUN_TEST(test_landed_filesystem);

    RUN_TEST(test_should_swap);
    RUN_TEST(test_swap_files);
    RUN_TEST(test_choose_flight_number);
    RUN_TEST(test_open_log_file);
    RUN_TEST(test_copy_out);

    remove_test_dir(TEST_DIR);
}
