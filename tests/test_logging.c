#include <ftw.h>
#include <limits.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <sys/stat.h>
#include <testing/unity.h>

#include "../telemetry/src/logging/logging.h"

#define TEST_DIR CONFIG_INSPACE_TELEMETRY_FLIGHT_FS "/testing"
#define TEST_FLIGHT_FS_DIR CONFIG_INSPACE_TELEMETRY_FLIGHT_FS "/fs_flight_test"
#define TEST_LANDED_FS_DIR CONFIG_INSPACE_TELEMETRY_LANDED_FS "/fs_landed_test"

#define TEST_SWAP_DIR TEST_DIR "/test_swap"
#define TEST_MISSION_NUM_DIR TEST_DIR "/test_flight_num"
#define TEST_OPEN_DIR TEST_DIR "/test_open"
#define TEST_COPY_DIR TEST_DIR "/test_copy"

/* Helpers */

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

static void create_file_with_contents(const char *filename, const char *data, const int len) {
    FILE *file_test = fopen(filename, "w");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(NULL, file_test, "Could not create a file");

    TEST_ASSERT_EQUAL_MESSAGE(len, fwrite(data, 1, len, file_test), "Could not write test data to a file");
    fflush(file_test);
    fsync(fileno(file_test));

    TEST_ASSERT_EQUAL_MESSAGE(0, fclose(file_test), "Could not to close a file");
}

static void check_file_contents(const char *filename, const char *data, const int len) {
    FILE *file_test = fopen(filename, "r+");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(NULL, file_test, "Could not reopen a file");

    char buffer[len + 1];
    int read_len = fread(buffer, 1, sizeof(buffer), file_test);
    TEST_ASSERT_EQUAL_MESSAGE(len, read_len, "Did not read same data from file that was written to");
    TEST_ASSERT_EQUAL_MEMORY(data, buffer, len);

    TEST_ASSERT_EQUAL_MESSAGE(0, fclose(file_test), "Could not to close a file");
}

static bool check_file_exists(const char *filename) {
    return access(filename, F_OK) == 0;
}

static void fmt_and_create_with_test_data(const char *fpath_fmt, int mission_num, int ser_num, char *test_data,
                                          int len) {
    char path[PATH_MAX];
    snprintf(path, sizeof(path), fpath_fmt, mission_num, ser_num);
    create_file_with_contents(path, test_data, len);
}

static void fmt_and_check_test_data(const char *fpath_fmt, int mission_num, int ser_num, char *test_data, int len) {
    char path[PATH_MAX];
    snprintf(path, sizeof(path), fpath_fmt, mission_num, ser_num);
    check_file_contents(path, test_data, len);
}

static void fmt_and_unlink(const char *fpath_fmt, int mission_num, int ser_num) {
    char path[PATH_MAX];
    snprintf(path, sizeof(path), fpath_fmt, mission_num, ser_num);
    TEST_ASSERT_EQUAL_MESSAGE(0, unlink(path), "Could not delete file");
}

/* Tests */

static void test_filesystem(const char *test_dir) {
    create_test_dir(test_dir);

    char filename[PATH_MAX];
    snprintf(filename, sizeof(filename), "%s/test_file", test_dir);

    char data[] = "TEST DATA";
    create_file_with_contents(filename, data, sizeof(data));

    FILE *file_test = fopen(filename, "r+");
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

    fflush(file_test);
    fsync(fileno(file_test));

    // Read truncated
    TEST_ASSERT_EQUAL_MESSAGE(0, fseek(file_test, 0, SEEK_SET), "Could not seek file to beginning");
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

static void test_choose_mission_number(void) {
    create_test_dir(TEST_FLIGHT_FS_DIR);
    create_test_dir(TEST_LANDED_FS_DIR);

    char fname_fmt[] = "test_format_%d_%d";
    char flight_fpath_fmt[] = TEST_FLIGHT_FS_DIR "/test_format_%d_%d";
    char land_fpath_fmt[] = TEST_LANDED_FS_DIR "/test_format_%d_%d";

    /* Test no files */
    TEST_ASSERT_EQUAL_MESSAGE(0, choose_mission_number(TEST_FLIGHT_FS_DIR, fname_fmt, TEST_LANDED_FS_DIR, fname_fmt),
                              "Did not get right mission number for empty dir");

    /* Test one file in one dir */
    fmt_and_create_with_test_data(flight_fpath_fmt, 0, 0, "", 0);
    TEST_ASSERT_EQUAL_MESSAGE(1, choose_mission_number(TEST_FLIGHT_FS_DIR, fname_fmt, TEST_LANDED_FS_DIR, fname_fmt),
                              "Did not get right mission number for dir with one entry");

    /* Test two files, with same mission number, in one dir */
    fmt_and_create_with_test_data(flight_fpath_fmt, 0, 1, "", 0);
    TEST_ASSERT_EQUAL_MESSAGE(1, choose_mission_number(TEST_FLIGHT_FS_DIR, fname_fmt, TEST_LANDED_FS_DIR, fname_fmt),
                              "Did not get right mission number for dir with two entries w same mission number");

    /* Test new highest mission number is in other dir */
    fmt_and_create_with_test_data(land_fpath_fmt, 1, 0, "", 0);
    TEST_ASSERT_EQUAL_MESSAGE(2, choose_mission_number(TEST_FLIGHT_FS_DIR, fname_fmt, TEST_LANDED_FS_DIR, fname_fmt),
                              "Did not get right mission number when both fs have files");

    /* Test same mission number in both partitions */
    fmt_and_create_with_test_data(flight_fpath_fmt, 1, 0, "", 0);
    TEST_ASSERT_EQUAL_MESSAGE(2, choose_mission_number(TEST_FLIGHT_FS_DIR, fname_fmt, TEST_LANDED_FS_DIR, fname_fmt),
                              "Did not get right mission number when both fs have same number");

    remove_test_dir(TEST_FLIGHT_FS_DIR);
    remove_test_dir(TEST_LANDED_FS_DIR);
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

static void check_copy_file(const char *from_filename, const char *to_filename) {
    char data[] = "TEST DATA";
    create_file_with_contents(from_filename, data, sizeof(data));
    copy_file(from_filename, to_filename);
    check_file_contents(to_filename, data, sizeof(data));
}

static void test_copy_file(void) {
    create_test_dir(TEST_FLIGHT_FS_DIR);
    create_test_dir(TEST_LANDED_FS_DIR);

    check_copy_file(TEST_FLIGHT_FS_DIR "/same_fs_from", TEST_FLIGHT_FS_DIR "/same_fs_to");
    check_copy_file(TEST_LANDED_FS_DIR "/same_fs_from", TEST_LANDED_FS_DIR "/same_fs_to");

    check_copy_file(TEST_FLIGHT_FS_DIR "/diff_fs_from", TEST_LANDED_FS_DIR "/diff_fs_to");
    check_copy_file(TEST_LANDED_FS_DIR "/diff_fs_from", TEST_FLIGHT_FS_DIR "/diff_fs_to");

    remove_test_dir(TEST_FLIGHT_FS_DIR);
    remove_test_dir(TEST_LANDED_FS_DIR);
}

static void check_sync_files(const char *from_dir, const char *to_dir) {
    char from_fname_format[] = "from_%d_%d";
    char from_fpath_format[PATH_MAX];
    snprintf(from_fpath_format, sizeof(from_fpath_format), "%s/%s", from_dir, from_fname_format);

    char to_fname_format[] = "to_%d_%d";
    char to_fpath_format[PATH_MAX];
    snprintf(to_fpath_format, sizeof(to_fpath_format), "%s/%s", to_dir, to_fname_format);

    TEST_ASSERT_EQUAL_MESSAGE(0, sync_files(from_dir, from_fname_format, to_fpath_format),
                              "Shouldn't have gotten an error syncing empty with empty dir");

    // Test syncing one file to an empty dir
    char data[] = "TEST DATA";
    int f1_mission_num = 0;
    int f1_ser_num = 0;
    fmt_and_create_with_test_data(from_fpath_format, f1_mission_num, f1_ser_num, data, sizeof(data));
    TEST_ASSERT_EQUAL_MESSAGE(0, sync_files(from_dir, from_fname_format, to_fpath_format),
                              "Could not sync files successfully");
    fmt_and_check_test_data(to_fpath_format, f1_mission_num, f1_ser_num, data, sizeof(data));

    // Test syncing if the file already exists
    fmt_and_create_with_test_data(from_fpath_format, f1_mission_num, f1_ser_num, data, sizeof(data));
    TEST_ASSERT_EQUAL_MESSAGE(0, sync_files(from_dir, from_fname_format, to_fpath_format),
                              "Could not sync files successfully");
    fmt_and_check_test_data(to_fpath_format, f1_mission_num, f1_ser_num, data, sizeof(data));

    // Test removing the original file, then checking that the synced file still exists afterwards
    fmt_and_unlink(from_fpath_format, f1_mission_num, f1_ser_num);
    TEST_ASSERT_EQUAL_MESSAGE(0, sync_files(from_dir, from_fname_format, to_fpath_format),
                              "Could not sync files successfully");
    fmt_and_check_test_data(to_fpath_format, f1_mission_num, f1_ser_num, data, sizeof(data));

    // Test syncing multiple files at the same time
    int f2_mission_num = 1;
    int f2_ser_num = 0;
    fmt_and_create_with_test_data(from_fpath_format, f2_mission_num, f2_ser_num, data, sizeof(data));

    int f3_mission_num = 1;
    int f3_ser_num = 1;
    fmt_and_create_with_test_data(from_fpath_format, f3_mission_num, f3_ser_num, data, sizeof(data));

    TEST_ASSERT_EQUAL_MESSAGE(0, sync_files(from_dir, from_fname_format, to_fpath_format),
                              "Could not sync files successfully");

    fmt_and_check_test_data(to_fpath_format, f1_mission_num, f1_ser_num, data, sizeof(data));
    fmt_and_check_test_data(to_fpath_format, f2_mission_num, f2_ser_num, data, sizeof(data));
    fmt_and_check_test_data(to_fpath_format, f3_mission_num, f3_ser_num, data, sizeof(data));
}

static void test_sync_files(void) {
    create_test_dir(TEST_FLIGHT_FS_DIR);
    create_test_dir(TEST_LANDED_FS_DIR);

    check_sync_files(TEST_FLIGHT_FS_DIR, TEST_LANDED_FS_DIR);

    remove_test_dir(TEST_FLIGHT_FS_DIR);
    remove_test_dir(TEST_LANDED_FS_DIR);
}

static void test_clean_dir(void) {
    create_test_dir(TEST_COPY_DIR);

    char fname_format[] = "test_%d_%d";

    // Test on an empty dir
    TEST_ASSERT_EQUAL_MESSAGE(0, clean_dir(TEST_COPY_DIR, fname_format), "Failed to clean empty directory");

    char data[] = "TEST DATA";
    char f1_path[] = TEST_COPY_DIR "/test_1_1";
    char f2_path[] = TEST_COPY_DIR "/test_2_2";
    char f3_path[] = TEST_COPY_DIR "/test_3_3";

    // Test on a dir with a file that doesn't match the format
    char non_matching_path[] = TEST_COPY_DIR "/other_fmt_1";
    create_file_with_contents(non_matching_path, data, sizeof(data));
    TEST_ASSERT_EQUAL_MESSAGE(0, clean_dir(TEST_COPY_DIR, fname_format), "Failed to clean directory with non-matching file");
    TEST_ASSERT_TRUE_MESSAGE(check_file_exists(non_matching_path), "Should not have deleted non-matching file");

    // Test on a dir with one entry
    create_file_with_contents(f1_path, data, sizeof(data));
    TEST_ASSERT_EQUAL_MESSAGE(0, clean_dir(TEST_COPY_DIR, fname_format), "Failed to directory with one entry");
    TEST_ASSERT_FALSE_MESSAGE(check_file_exists(f1_path), "Should have cleaned up single entry");

    // Test on a dir with multiple entries
    create_file_with_contents(f1_path, data, sizeof(data));
    create_file_with_contents(f2_path, data, sizeof(data));
    create_file_with_contents(f3_path, data, sizeof(data));

    TEST_ASSERT_EQUAL_MESSAGE(0, clean_dir(TEST_COPY_DIR, fname_format), "Failed to directory with multiple entries");
    int num_exists = check_file_exists(f1_path);
    num_exists += check_file_exists(f2_path);
    num_exists += check_file_exists(f3_path);
    TEST_ASSERT_EQUAL_MESSAGE(0, num_exists, "Should have had 0 files left after cleaning dir with 3 entries");

    remove_test_dir(TEST_COPY_DIR);
}

void test_logging(void) {
    create_test_dir(TEST_DIR);

    RUN_TEST(test_flight_filesystem);
    RUN_TEST(test_landed_filesystem);

    RUN_TEST(test_should_swap);
    RUN_TEST(test_swap_files);
    RUN_TEST(test_choose_mission_number);
    RUN_TEST(test_open_log_file);
    RUN_TEST(test_copy_file);
    RUN_TEST(test_sync_files);
    RUN_TEST(test_clean_dir);

    remove_test_dir(TEST_DIR);
}
