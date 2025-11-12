#include <ArduinoJson.h>  // by Benoit Blanchon 6.21.3
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <FS.h>
#include <HTTPClient.h>
#include <limits>
#include <map>
#include <SPIFFS.h>
#include <SPI.h>
#include <TFT_eSPI.h>  // by Bodmer 2.5.0 installed manual
#include <unordered_map>
#include <vector>
#include <WiFiManager.h>  // by tzapu 2.0.16

//#define DEBUG_SERIAL_WIEN_MONITOR 0
#define TFT_BG tft.color565(0, 5, 0)
#define TFT_TEXT tft.color565(255, 240, 93)
#define URL_BASE \
  "https://www.wienerlinien.at/ogd_realtime/monitor?" \
  "activateTrafficInfo=stoerunglang&rbl="
#define JSON_CONFIG_FILE "/sample_config.json"

// Forvard declaration
class Screen;
class TraficManager;
class TraficClock;

struct Config;
struct ConfigFileHandler;
struct GlobalSettings;
struct Monitor;
struct ScreenEntity;
struct StringDatase;

std::vector<Monitor> GetMonitorsFromJson(const String&);
std::vector<Monitor> GetFilteredMonitors(const std::vector<Monitor>&, const String&);
std::vector<std::vector<Monitor>> GetMonitorsPerDisplayLine(const std::vector<Monitor>&, const Config&);
std::vector<std::vector<Monitor>> GetMonitorsPerDisplayLineWithSeparateLine3(const std::vector<Monitor>&, const std::vector<Monitor>&, const Config&);
std::vector<String> GetSplittedStrings(String, char);
String FixJsonMistake(String);
String GetRandomString(int);
String GetJson(const String&);
template<typename T> std::vector<T> cyclicSubset(const std::vector<T>&, size_t, size_t);
void PrintSystemInfo();
void PrintDebugMonitor(const Monitor&);
void PrintDegbugMonitors(const std::vector<Monitor>&);
void ResetActionsTask(void*);
void ScreenUpdateTask(void*);
void UpdateDataTask(void*);
void WiFiManagerTask();

// Global variables
SemaphoreHandle_t dataMutex;
TFT_eSPI tft = TFT_eSPI();
Screen* p_screen = nullptr;
TraficManager* pTraficManager = nullptr;


/**
 * @struct StringDatabase
 *
 * @brief A utility struct for managing string prompts and instructions.
 */
struct StringDatabase {
private:
  /**
     * @brief The prompt for finding RBL/Stop ID.
     */
  const static constexpr char* RBLPrompt =
    "Find your RBL on https://till.mabe.at/rbl/."
    "<br>Example: \"49\".<br><br><b>RBL/Stop ID:</b>";

  /**
     * @brief The prompt for filtering lines.
     */
  const static constexpr char* LineFilterPrompt =
    "<i>Optional.</i>"
    "Filter the lines to show by comma-separating the directions."
    "If empty, all directions will be shown.<br>"
    "Example: \"D,2,U2Z\".<br><br>"
    "<b>Filter lines:</b>";

  /**
     * @brief The prompt for Line 1.
     */
  const static constexpr char* Line1Prompt =
    "<i>Optional.</i>"
    "Specify which train/tram line to show on display line 1."
    "If empty, line 1 will show mixed departures.<br>"
    "Example: \"25\".<br><br>"
    "<b>Display Line 1:</b>";

  /**
     * @brief The prompt for Line 2.
     */
  const static constexpr char* Line2Prompt =
    "<i>Optional.</i>"
    "Specify which train/tram line to show on display line 2."
    "If empty, line 2 will show mixed departures.<br>"
    "Example: \"26\".<br><br>"
    "<b>Display Line 2:</b>";

  /**
     * @brief The prompt for Line 3.
     */
  const static constexpr char* Line3Prompt =
    "<i>Optional.</i>"
    "Specify which train/tram line to show on display line 3."
    "If empty, line 3 will show mixed departures.<br>"
    "Example: \"U6\".<br><br>"
    "<b>Display Line 3:</b>";

  /**
     * @brief The prompt for Line 3 RBL.
     */
  const static constexpr char* Line3RBLPrompt =
    "<i>Optional.</i>"
    "Separate Stop ID (RBL) for Line 3 only. "
    "If empty, uses main RBL above.<br>"
    "Example: \"123\".<br><br>"
    "<b>Line 3 Stop ID (RBL):</b>";

  /**
     * @brief The prompt for Line 3 Filter.
     */
  const static constexpr char* Line3FilterPrompt =
    "<i>Optional.</i>"
    "Filter for Line 3 stop only. "
    "If empty, uses main filter above.<br>"
    "Example: \"U6,26A\".<br><br>"
    "<b>Line 3 Filter:</b>";

public:
  /**
     * @brief Get the Wi-Fi SSID string.
     * @return The Wi-Fi SSID string.
     */
  static String GetWiFissid() {
    return "Wien Transport 🚇⏱️";
  }

  /**
     * @brief Get the instructions text.
     * @return The instructions text.
     */
  static String GetInstructionsText() {
    String instruction_start =
      "Wait a few seconds or:\n"
      "1) Take a smartphone.\n"
      "2) Connect to Wi-Fi:\n"
      " \"";
    String instruction_end = "\"\n3) And follow prompts. (^_^)\n";
    return instruction_start + GetWiFissid() + instruction_end;
  }

  /**
     * @brief Get the RBL/Stop ID prompt.
     * @return The RBL/Stop ID prompt.
     */
  static String GetRBLPrompt() {
    return String(RBLPrompt);
  }

  /**
     * @brief Get the line filter prompt.
     * @return The line filter prompt.
     */
  static String GetLineFilterPrompt() {
    return String(LineFilterPrompt);
  }

  /**
     * @brief Get the Line 1 prompt.
     * @return The Line 1 prompt.
     */
  static String GetLine1Prompt() {
    return String(Line1Prompt);
  }

  /**
     * @brief Get the Line 2 prompt.
     * @return The Line 2 prompt.
     */
  static String GetLine2Prompt() {
    return String(Line2Prompt);
  }

  /**
     * @brief Get the Line 3 prompt.
     * @return The Line 3 prompt.
     */
  static String GetLine3Prompt() {
    return String(Line3Prompt);
  }

  /**
     * @brief Get the Line 3 RBL prompt.
     * @return The Line 3 RBL prompt.
     */
  static String GetLine3RBLPrompt() {
    return String(Line3RBLPrompt);
  }

  /**
     * @brief Get the Line 3 Filter prompt.
     * @return The Line 3 Filter prompt.
     */
  static String GetLine3FilterPrompt() {
    return String(Line3FilterPrompt);
  }

  /**
     * @brief Get the prompt for specifying the number of lines to show.
     * @param min The minimum number of lines.
     * @param max The maximum number of lines.
     * @param def The default number of lines.
     * @return The prompt for specifying the number of lines to show.
     */
  static String GetLineCountPrompt(int min, int max, int def) {
    String range = GetFormatRange(min, max);
    String result =
      "How many shown depatures do you want to show at the same time on monitor "
      + range + "? (Default: "
      + String(def) + "). <br>"
                      "Example: \""
      + String(def) + "\".<br><br>"
                      "<b>Lines to show:</b>";
    return result;
  }

private:
  /**
     * @brief Format a range of values as a string.
     * @param min The minimum value.
     * @param max The maximum value.
     * @return The formatted range string.
     *
     * Example: GetFormatRange(1, 3) returns "1, 2, or 3".
     */
  static String GetFormatRange(int min, int max) {
    if (min > max) {
      // Swap min and max if min is greater than max
      int temp = min;
      min = max;
      max = temp;
    }

    String result = "";

    if (min == max) {
      result += String(min);
    } else if (max - min == 1) {
      result += String(min) + " or " + String(max);
    } else {
      for (int i = min; i < max; i++) {
        result += String(i) + ", ";
      }
      result += "or " + String(max);
    }

    return result;
  }
};

struct Config {
  static const int cnt_min_lines = 1;
  static const int cnt_max_lines = 3;
  static const int cnt_default_lines = 2;

  /**
     * @brief Default constructor.
     *
     * Sets the number of lines to the default of cnt_default_lines.
     */
  explicit Config()
    : cnt_lines(cnt_default_lines) {
    line_1_name = "";
    line_2_name = "";
    line_3_name = "";
    line_3_rbl = "";
    line_3_filter = "";
  }

  /**
     * @brief Copy constructor.
     *
     * Copies all of the members of the other Config object to this one.
     *
     * @param other The Config object to copy from.
     */
  Config(const Config& other)
    : cnt_lines(other.cnt_lines),
      lines_filter(other.lines_filter),
      lines_rbl(other.lines_rbl),
      line_1_name(other.line_1_name),
      line_2_name(other.line_2_name),
      line_3_name(other.line_3_name),
      line_3_rbl(other.line_3_rbl),
      line_3_filter(other.line_3_filter) {}
  /**
     * @brief Assignment operator.
     *
     * Copies all of the members of the other Config object to this one.
     *
     * @param other The Config object to copy from.
     * @return A reference to this Config object.
     */
  Config& operator=(const Config& other) {
    if (this != &other) {
      lines_filter = other.lines_filter;
      lines_rbl = other.lines_rbl;
      cnt_lines = other.cnt_lines;
      line_1_name = other.line_1_name;
      line_2_name = other.line_2_name;
      line_3_name = other.line_3_name;
      line_3_rbl = other.line_3_rbl;
      line_3_filter = other.line_3_filter;
    }
    return *this;
  }

  /**
     * @brief Equality operator.
     *
     * Compares two Config objects for equality.
     *
     * @param other The Config object to compare to.
     * @return True if the two Config objects are equal, false otherwise.
     */
  bool operator==(const Config& other) const {
    return lines_filter == other.lines_filter
           && lines_rbl == other.lines_rbl
           && cnt_lines == other.cnt_lines
           && line_1_name == other.line_1_name
           && line_2_name == other.line_2_name
           && line_3_name == other.line_3_name
           && line_3_rbl == other.line_3_rbl
           && line_3_filter == other.line_3_filter;
  }
  /**
     * @brief Inequality operator.
     *
     * Compares two Config objects for inequality.
     *
     * @param other The Config object to compare to.
     * @return True if the two Config objects are not equal, false otherwise.
     */
  bool operator!=(const Config& other) const {
    return !(*this == other);
  }

  /**
     * @brief Sets the number of lines.
     *
     * Valid values are from cnt_min_lines to cnt_max_lines.
     * If an invalid value is passed, the default
     * value of cnt_default_lines will be used.
     *
     * @param count The number of lines to set.
     */
  void SetLinesCount(int c) {
    cnt_lines = verifyLinesCountInput(c);
  }
  /**
     * @brief Sets the number of lines.
     *
     * Valid values are from cnt_min_lines to cnt_max_lines.
     * If an invalid value is passed, the default
     * value of cnt_default_lines will be used.
     *
     * @param str The number of lines to set as a string.
     */
  void SetLinesCount(const String& c) {
    cnt_lines = verifyLinesCountInput(c.toInt());
  }

  /**
     * @brief Gets the number of lines as an integer.
     *
     * @return The number of lines.
     */
  int GetLinesCountAsInt() const {
    return cnt_lines;
  }
  /**
     * @brief Gets the number of lines as a string.
     *
     * @return The number of lines as a string.
     */
  const String GetLinesCountAsString() const {
    return String(cnt_lines);
  }

  /**
     * @brief Sets the lines filter.
     *
     * @param str The lines filter to set.
     */
  void SetLinesFilter(const String& str) {
    lines_filter = str;
  }
  /**
     * @brief Gets the lines filter as a string.
     *
     * @return The lines filter as a string.
     */
  const String& GetLinesFilterAsString() const {
    return lines_filter;
  }

  /**
     * @brief Sets the lines RBL.
     *
     * @param str The lines RBL to set.
     */
  void SetLinesRBL(const String& str) {
    lines_rbl = str;
  }
  /**
     * @brief Sets the lines RBL.
     *
     * @param str The lines RBL to set as an integer.
     */
  void SetLinesRBL(int str) {
    lines_rbl = String(str, DEC);
  }
  /**
     * @brief Gets the lines RBL as a string.
     *
     * @return The lines RBL as a string.
     */
  const String& GetLinesRblAsString() const {
    return lines_rbl;
  }
  /**
     * @brief Gets the lines RBL as an integer.
     *
     * @return The lines RBL as an integer.
     */
  int GetLinesRblAsInt() {
    return lines_rbl.toInt();
  }

  /**
     * @brief Sets the line name for a specific display line.
     *
     * @param line_idx The line index (1, 2, or 3).
     * @param name The line name (e.g., "25", "U6", "D").
     */
  void SetLineName(int line_idx, const String& name) {
    switch (line_idx) {
      case 1:
        line_1_name = name;
        break;
      case 2:
        line_2_name = name;
        break;
      case 3:
        line_3_name = name;
        break;
    }
  }

  /**
     * @brief Gets the line name for a specific display line.
     *
     * @param line_idx The line index (1, 2, or 3).
     * @return The line name for that display line.
     */
  const String& GetLineName(int line_idx) const {
    switch (line_idx) {
      case 1:
        return line_1_name;
      case 2:
        return line_2_name;
      case 3:
        return line_3_name;
      default:
        return line_1_name;
    }
  }

  /**
     * @brief Sets the RBL for line 3.
     *
     * @param str The RBL to set.
     */
  void SetLine3RBL(const String& str) {
    line_3_rbl = str;
  }

  /**
     * @brief Gets the RBL for line 3.
     *
     * @return The RBL for line 3.
     */
  const String& GetLine3RBL() const {
    return line_3_rbl;
  }

  /**
     * @brief Sets the filter for line 3.
     *
     * @param str The filter to set.
     */
  void SetLine3Filter(const String& str) {
    line_3_filter = str;
  }

  /**
     * @brief Gets the filter for line 3.
     *
     * @return The filter for line 3.
     */
  const String& GetLine3Filter() const {
    return line_3_filter;
  }

private:
  /**
     * @brief Verifies that the lines count input is valid.
     *
     * Valid values are from cnt_min_lines to cnt_max_lines.
     * If an invalid value is passed, the default
     * value of cnt_default_lines will be used.
     *
     * @param count The lines count input.
     * @return The lines count input, if it is valid. Otherwise, the default
     * count.
     */
  static int verifyLinesCountInput(int count) {
    switch (count) {
      case cnt_min_lines ... cnt_max_lines:
        return count;
      default:
        return cnt_default_lines;
    }
  }

private:
  ///< The number of lines of text that can be displayed in each idx_row.
  int cnt_lines;

  ///< The raw lines filter. Example "D,2,6".
  String lines_filter;

  ///< The lines RBL.
  String lines_rbl;

  ///< Line names for each display line (1, 2, 3)
  String line_1_name;
  String line_2_name;
  String line_3_name;

  ///< Separate RBL for line 3 (optional)
  String line_3_rbl;

  ///< Separate filter for line 3 (optional)
  String line_3_filter;
};

/**
 * @brief Class for handling configuration files.
 *
 * This class provides functions for saving and loading configuration files to
 * and from SPIFFS.
 */
struct ConfigFileHandler {
  /**
     * @brief Saves the configuration file to SPIFFS.
     *
     * @param cfg The configuration to save.
     */
  static void SaveConfigFile(Config& cfg) {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
    Serial.println(F("Saving config"));
#endif
    StaticJsonDocument<512> json;
    json["lines_filter"] = cfg.GetLinesFilterAsString();
    json["rbl_id"] = cfg.GetLinesRblAsString();
    json["lines_count"] = cfg.GetLinesCountAsString();
    json["line_1_name"] = cfg.GetLineName(1);
    json["line_2_name"] = cfg.GetLineName(2);
    json["line_3_name"] = cfg.GetLineName(3);
    json["line_3_rbl"] = cfg.GetLine3RBL();
    json["line_3_filter"] = cfg.GetLine3Filter();

    File file_config = SPIFFS.open(JSON_CONFIG_FILE, "w");
    if (!file_config) {
      Serial.println("failed to open config file for writing");
    }

    serializeJsonPretty(json, Serial);
    if (serializeJson(json, file_config) == 0) {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
      Serial.println(F("Failed to write to file"));
#endif
    }
    file_config.close();
  }

  /**
     * @brief Loads the configuration file from SPIFFS.
     *
     * @param cfg The configuration to load into.
     * @return True if the config file was loaded successfully, false otherwise.
     */
  static bool LoadConfigFile(Config& cfg) {
    // clean FS, for testing
    // SPIFFS.format();
    // read configuration from FS json
#ifdef DEBUG_SERIAL_WIEN_MONITOR
    Serial.println("mounting FS...");
#endif

    if (SPIFFS.begin(false) || SPIFFS.begin(true)) {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
      Serial.println("mounted file system");
#endif
      if (SPIFFS.exists(JSON_CONFIG_FILE)) {
        // file exists, reading and loading
#ifdef DEBUG_SERIAL_WIEN_MONITOR
        Serial.println("reading config file");
#endif
        File configFile = SPIFFS.open(JSON_CONFIG_FILE, "r");
        if (configFile) {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
          Serial.println("opened config file");
#endif
          StaticJsonDocument<512> json;
          DeserializationError error = deserializeJson(json, configFile);
          serializeJsonPretty(json, Serial);
          if (!error) {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
            Serial.println("\nparsed json");
#endif

            cfg.SetLinesFilter(json["lines_filter"].as<String>());
            cfg.SetLinesRBL(json["rbl_id"].as<String>());
            cfg.SetLinesCount(json["lines_count"].as<String>());
            cfg.SetLineName(1, json["line_1_name"].as<String>());
            cfg.SetLineName(2, json["line_2_name"].as<String>());
            cfg.SetLineName(3, json["line_3_name"].as<String>());
            cfg.SetLine3RBL(json["line_3_rbl"].as<String>());
            cfg.SetLine3Filter(json["line_3_filter"].as<String>());
            Serial.println("\nparsed json end");
            return true;
          } else {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
            Serial.println("failed to load json config");
#endif
          }
        }
        configFile.close();
      }
    } else {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
      Serial.println("failed to mount FS");
#endif
    }
    // end read
    return false;
  }

  /**
     * @brief Deletes the configuration file from SPIFFS.
     */
  static void DeleteConfigFile() {
    if (SPIFFS.begin()) {
      if (SPIFFS.exists(JSON_CONFIG_FILE)) {
        if (SPIFFS.remove(JSON_CONFIG_FILE)) {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
          Serial.println("Config file successfully deleted");
#endif
        } else {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
          Serial.println("Failed to delete config file");
#endif
        }
      } else {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
        Serial.println("Config file does not exist");
#endif
      }
      SPIFFS.end();
    } else {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
      Serial.println("Failed to initialize SPIFFS");
#endif
    }
  }
};

struct GlobalSettings {
  ///< Number of lines of text that can be displayed on each idx_row of the
  ///< screen.
  static constexpr int cnt_screen_lines_per_rows = 2;

  ///< The number queue. Indicates how many countdown shows per cycle.
  static constexpr int cnt_shows_countdows = 2;

  ///< Number of seconds to press the soft reset button to restart the device.
  static constexpr int sec_press_soft_resset_button = 5;

  ///< Number of seconds to press the hard reset button to restart the device.
  static constexpr int sec_press_hard_resset_button = 30;

  ///< GPIO pin that the reset button is connected to.
  static constexpr int pin_reset_button = 0;

  ///< Number of milliseconds to delay before resetting the device if an error.
  static constexpr int ms_error_resset_delay = 10 * 1000;

  ///< Number of milliseconds to reset esp.
  static constexpr int ms_reboot_interval = 3 * 60 * 60 * 1000;

  ///< Number of milliseconds to delay between checking the reset button state.
  static constexpr int ms_task_delay_resset_button = 1000;

  ///< Number of milliseconds to delay between updating the data.
  static constexpr int ms_task_delay_data_update = 60 * 1000;

  ///< Number of milliseconds to delay between updating the screen.
  static constexpr int ms_task_delay_screen_update = 10;

  ///< real count down can be faster thet real ms_task_delay_data_update
  ///< this offset time help meke more smoth transition
  static constexpr int ms_additional_time_for_countdown = 50;

  ///< Size of font that show in first second plug in
  static constexpr int size_start_instruction_font = 4;

  ///< how much pixels scroll per frame
  static constexpr int px_scrool_per_frame = 2;

private:
  ///< The configuration object.
  Config config;

  ///< Whether or not the configuration file has been loaded.
  bool is_config_loaded = false;

public:
  /**
     * @brief Gets the global settings.
     *
     * @return A reference to the global settings object.
     */
  const Config& GetConfig() {
    if (!is_config_loaded) {
      is_config_loaded = ConfigFileHandler::LoadConfigFile(config);
    }
    return config;
  }

  /**
     * @brief Sets the global settings.
     *
     * @param new_config The new global settings.
     */
  void SetConfig(const Config& new_config) {
    config = new_config;
    ConfigFileHandler::SaveConfigFile(config);
    is_config_loaded = true;
  }

  /**
     * @brief Deletes the configuration file.
     */
  void DeleteFile() {
    ConfigFileHandler::DeleteConfigFile();
  }
} global_settings;


/**
 * @brief Manages API call caching and rate limiting for RBL data fetches.
 *
 * This class ensures:
 * - Only one API call per unique RBL per minute
 * - Cached responses are shared between lines using the same RBL
 * - Sequential API calls with delay when fetching multiple RBLs
 */
class ApiCacheManager {
private:
  struct CachedData {
    String json_response;
    unsigned long timestamp_ms;
    bool is_valid;

    CachedData() : json_response(""), timestamp_ms(0), is_valid(false) {}
  };

  std::map<String, CachedData> cache;
  static constexpr unsigned long CACHE_DURATION_MS = 60000; // 1 minute
  static constexpr unsigned long SEQUENTIAL_DELAY_MS = 3000; // 3 seconds between API calls
  unsigned long last_api_call_ms = 0;

public:
  /**
   * @brief Fetches JSON data for an RBL, using cache if valid.
   * @param rbl_id The RBL identifier to fetch.
   * @param force_refresh If true, bypasses cache and fetches fresh data.
   * @return JSON string response.
   */
  String FetchRBL(const String& rbl_id, bool force_refresh = false) {
    if (rbl_id.isEmpty()) {
      return "";
    }

    unsigned long current_ms = millis();

    // Check if we have valid cached data
    if (!force_refresh && cache.count(rbl_id) > 0) {
      CachedData& cached = cache[rbl_id];
      unsigned long age_ms = current_ms - cached.timestamp_ms;

      if (cached.is_valid && age_ms < CACHE_DURATION_MS) {
        Serial.print("Using cached data for RBL ");
        Serial.print(rbl_id);
        Serial.print(" (age: ");
        Serial.print(age_ms / 1000);
        Serial.println(" seconds)");
        return cached.json_response;
      }
    }

    // Enforce sequential delay between API calls
    unsigned long time_since_last_call = current_ms - last_api_call_ms;
    if (last_api_call_ms > 0 && time_since_last_call < SEQUENTIAL_DELAY_MS) {
      unsigned long delay_needed = SEQUENTIAL_DELAY_MS - time_since_last_call;
      Serial.print("Delaying API call for ");
      Serial.print(delay_needed);
      Serial.println(" ms to respect rate limits");
      delay(delay_needed);
    }

    // Fetch fresh data
    Serial.print("Fetching fresh data for RBL: ");
    Serial.println(rbl_id);
    String json_data = GetJson(rbl_id);
    last_api_call_ms = millis();

    // Update cache
    CachedData& cached = cache[rbl_id];
    cached.json_response = json_data;
    cached.timestamp_ms = last_api_call_ms;
    cached.is_valid = !json_data.isEmpty();

    return json_data;
  }

  /**
   * @brief Invalidates cached data for a specific RBL.
   * @param rbl_id The RBL identifier to invalidate.
   */
  void InvalidateCache(const String& rbl_id) {
    if (cache.count(rbl_id) > 0) {
      cache[rbl_id].is_valid = false;
    }
  }

  /**
   * @brief Clears all cached data.
   */
  void ClearCache() {
    cache.clear();
  }

  /**
   * @brief Gets the age of cached data for an RBL in milliseconds.
   * @param rbl_id The RBL identifier.
   * @return Age in milliseconds, or ULONG_MAX if not cached.
   */
  unsigned long GetCacheAge(const String& rbl_id) {
    if (cache.count(rbl_id) > 0 && cache[rbl_id].is_valid) {
      return millis() - cache[rbl_id].timestamp_ms;
    }
    return ULONG_MAX;
  }
} api_cache;


class SmartWatch {
public:
  SmartWatch(const char* function_name)
    : function_name(function_name), start_millis(millis()) {}

  unsigned long GetExecution_ms() {
    unsigned long end_millis = millis();
    return end_millis - start_millis;
  }
  ~SmartWatch() {
#if 1
    Serial.print("Function '");
    Serial.print(function_name);
    Serial.print("' executed in ");
    Serial.print(GetExecution_ms());
    Serial.println(" milliseconds.");
#endif
  }

private:
  const char* function_name;
  unsigned long start_millis;
};

struct Monitor {
  String name;         // Line name 2, 72, D etc.
  String towards;      // Tram directions.
  String description;  // Line alarm
  std::vector<int> countdown;
};

struct ScreenEntity {
  String right_txt;  // Line name 2, 72, D etc.
  std::vector<String> lines;
  String left_txt;
};


/**
 * @brief The `Screen` class represents a screen with multiple rows, each
 * displaying text and countdown information.
 *
 * This class is designed for use with TFT displays and provides methods for
 * setting and drawing text on each idx_row, as well as managing scrolling text
 * and drawing separator lines.
 */
class Screen {
public:
  /**
     * @brief Constructor to initialize a `Screen` object with the specified
     * number of rows and lines per idx_row.
     *
     * @param cnt_rows The number of rows on the screen.
     * @param cnt_lines_in_row The number of lines of text that can be displayed
     * in each idx_row.
     */
  Screen(int cnt_rows, int nLinesInRowCount)
    : px_max_width_name_text(0),
      px_max_width_countdown_text(0),
      px_separate_line_height(3),
      px_margin_lines(2),
      cnt_rows(cnt_rows),
      cnt_lines_in_row(nLinesInRowCount),
      px_margin(8),
      px_min_text_sprite(std::numeric_limits<int>::max()) {
    vec_scrolls_coords.resize(cnt_rows, std::vector<int>(nLinesInRowCount, 0));
    vec_init_scrolls_coords.resize(cnt_rows,
                                   std::vector<bool>(nLinesInRowCount, false));
  }


public:
  /**
     * @brief Sets the content for a specific idx_row on the screen and updates
     * its display.
     *
     * @param monitor The `Monitor` object containing the text and countdown
     * information for the idx_row.
     * @param idx_row The idx_row index to set the content for.
     */
  void SetRow(const ScreenEntity& monitor, int idx_row) {
    if (idx_row > cnt_rows - 1) {
      return;
    }
    const GFXfont* p_font = &FreeSansBold24pt7b;

    // Calculate max Sides text block size.
    int nameTextWidth_px = CalculateFontWidth_px(p_font, monitor.right_txt);
    SetMaxNameTextWidth_px(nameTextWidth_px);
    int countdownTextWidth_px = CalculateFontWidth_px(p_font, monitor.left_txt);
    SetMaxCountdownTextWidth_px(countdownTextWidth_px);

    // Draw Text;
    DrawName(monitor.right_txt, idx_row, p_font);
    DrawCountdown(monitor.left_txt, idx_row, p_font);
    DrawMiddleText(monitor.lines, idx_row);

    // Draw Separet Lines
    drawLines();
  }
  bool IsEnoughSpaceForMiddleText(const String& str) {
    const GFXfont* p_font = &FreeSansBold12pt7b;
    return GetMinTextSprite_px() >= CalculateFontWidth_px(p_font, str);
  }


  void PrintCordDebug() {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
    // Print the contents of the vectors
    for (int i = 0; i < vec_scrolls_coords.size(); i++) {
      Serial.print("[");
      for (int j = 0; j < vec_scrolls_coords[i].size(); j++) {
        Serial.print(vec_scrolls_coords[i][j]);
        if (j != vec_scrolls_coords[i].size() - 1) {
          Serial.print(", ");
        }
      }
      Serial.print("]");
    }

    for (int i = 0; i < vec_init_scrolls_coords.size(); i++) {
      Serial.print("[");
      for (int j = 0; j < vec_init_scrolls_coords[i].size(); j++) {
        Serial.print(vec_init_scrolls_coords[i][j]);
        if (j != vec_init_scrolls_coords[i].size() - 1) {
          Serial.print(", ");
        }
      }
      Serial.print("]");
    }
    Serial.println("");
#endif
  }

  /**
     * @brief Converts a single German string to Latin alphabet.
     *
     * This function takes a single German string and converts it from German
     * characters (e.g., ä, Ä, ö, Ö, ü, Ü, ß) to their Latin alphabet equivalents
     * (a, A, o, O, u, U, s).
     *
     * @param input The German string to be converted.
     *
     * @return The converted string in Latin alphabet.
     */
  static String ConvertGermanToLatin(String input) {
    // TODO: optimize it.
    input.replace("ä", "a");
    input.replace("Ä", "A");
    input.replace("ö", "o");
    input.replace("Ö", "O");
    input.replace("ü", "u");
    input.replace("Ü", "U");
    input.replace("ß", "ss");
    return input;
  }


  int GetMinTextSprite_px() {
    return px_min_text_sprite;
  }

private:
  /**
     * @brief Converts a vector of German strings to Latin alphabet.
     *
     * Takes a vector of German strings and converts each string from
     * German characters (e.g., ä, Ä, ö, Ö, ü, Ü, ß) to their Latin alphabet
     * equivalents (a, A, o, O, u, U, s). The converted strings are then returned
     * in a new vector.
     *
     * @param vecGerman A vector of German strings to be converted.
     *
     * @return A vector of strings in Latin alphabet.
     */
  std::vector<String> ConvertGermanToLatin(
    const std::vector<String>& vecGerman) {
    std::vector<String> output;
    for (auto& str : vecGerman) {
      output.push_back(ConvertGermanToLatin(str));
      // Serial.println(ConvertGermanToLatin(str));
    }
    return output;
  }


private:
  /**
     * @brief Draws the countdown text on the screen for a specific idx_row.
     *
     * @param countdown The countdown text to be displayed.
     * @param idx_row The idx_row index where the countdown text should be drawn.
     * @param p_font The pointer to the p_font to be used for rendering the text.
     */
  void DrawCountdown(const String& countdown, int idx_row,
                     const GFXfont* pFont) const {
    int px_font_width = CalculateFontWidth_px(pFont, countdown);

    int px_dx_countdown_margin = tft.width() - px_font_width - px_margin;

    DrawTextOnSprite(countdown, idx_row, px_dx_countdown_margin, 0, pFont,
                     GetMaxCountdownTextWidth_px());
  }


  /**
     * @brief Draws the name text on the screen for a specific idx_row.
     *
     * @param name The name text to be displayed.
     * @param idx_row The idx_row index where the name text should be drawn.
     * @param p_font The pointer to the p_font to be used for rendering the text.
     */
  void DrawName(const String& name, int row, const GFXfont* pFont) const {
    int dxNameMargin_px = px_margin;
    DrawTextOnSprite(name, row, dxNameMargin_px, 0, pFont,
                     GetMaxNameTextWidth_px());
  }


  /**
     * @brief Sets and draws the middle lines and text content for a specific
     * idx_row on the screen.
     *
     * @param vec_text_lines The vector of text lines to be displayed in the
     * idx_row.
     * @param idx_row The idx_row index where the text content should be set and
     * drawn.
     */
  void DrawMiddleText(const std::vector<String>& vec_text_lines, int idx_row) {

    const GFXfont* p_font = &FreeSansBold12pt7b;

    // Calculate dimensions
    auto& vec_scroll_cord = vec_scrolls_coords[idx_row];
    auto& is_init_cords = vec_init_scrolls_coords[idx_row];
    int px_width_countdown = GetMaxCountdownTextWidth_px();
    int px_width_stopcode = GetMaxNameTextWidth_px();
    int px_height_font = CalculatefontHeight_px(p_font);
    // Minus two margins for name and two margins for countdown
    // 4 is count margin around Coundown and Stop code
    int px_width =
      tft.width() - px_width_countdown - px_width_stopcode - px_margin * 4;

    SetMinTextSprite_px(px_width);

    // Create and configure sprite
    TFT_eSprite sprite(&tft);
    sprite.createSprite(px_width, px_height_font);
    // sprite.setTextSize(0);

    int cnt_lines_actual = static_cast<int>(vec_text_lines.size());
    for (int i = 0; i < std::min(cnt_lines_in_row, cnt_lines_actual); ++i) {
      int px_height_full = static_cast<double>(tft.height() / cnt_rows);
      int dy = CalculateLineDistance_px(px_height_full, px_margin_lines,
                                        px_height_font, cnt_lines_in_row, i);
      int px_full_string =
        CalculateFontWidth_px(p_font, vec_text_lines[i].c_str());

      // Reset to the right edge of the screen
      if (vec_scroll_cord[i] < (px_full_string * -1)) {
        vec_scroll_cord[i] = sprite.width();
      }


      if (px_full_string > px_width) {
        //scroll text 2 is mean px per frame
        vec_scroll_cord[i] -= global_settings.px_scrool_per_frame;
        if (!is_init_cords[i] && i > 0 && vec_text_lines[i].length()) {
          // Scroll if text is bigger than available space
          vec_scroll_cord[i] = sprite.width();
          is_init_cords[i] = true;
        }
      } else if (px_full_string < vec_scroll_cord[i]) {
        // reset sscrolls cord
        vec_scroll_cord[i] = 0;
        is_init_cords[i] = false;
      }

      // Draw text on the sprite
      sprite.setTextColor(TFT_TEXT);
      sprite.fillSprite(TFT_BG);
      sprite.setFreeFont(p_font);
      sprite.drawString(vec_text_lines[i].c_str(), vec_scroll_cord[i], 0);

      // Push sprite to display
      int x_cord = px_margin + px_margin + px_width_stopcode;
      int y_cord = dy + ((tft.height() / cnt_rows) * idx_row);
      sprite.pushSprite(x_cord, y_cord);
    }
    sprite.deleteSprite();
  }


  /**
     * @brief Draws text on a sprite and pushes it to the screen at the specified
     * coordinates.
     *
     * @param text The text to be drawn on the sprite.
     * @param idx_row The idx_row index where the text should be displayed.
     * @param x The X-coordinate where the text should be drawn.
     * @param y The Y-coordinate where the text should be drawn.
     * @param p_font The pointer to the p_font to be used for rendering the text.
     */
  void DrawTextOnSprite(const String& text, int idx_row, int x, int y,
                        const GFXfont* p_font, int px_max_width) const {
    const int px_width_sprite = CalculateFontWidth_px(p_font, text);
    const int px_height_font = CalculatefontHeight_px(p_font);
    const int px_height_full = tft.height() / cnt_rows;
    const int dy =
      CalculateLineDistance_px(px_height_full, 0, px_height_font, 1, 0);

    TFT_eSprite sprite(&tft);
    TFT_eSprite bg_left_sprite(&tft);
    TFT_eSprite bg_right_sprite(&tft);
    // TFT_eSprite bg_sprite(&tft);  // Separate sprite for squares

    // Set background colors based on DEBUG mode
    uint16_t color_left, color_middle, color_right;
    //#define DEBUG_DrawTextOnSprite
#ifdef DEBUG_DrawTextOnSprite
    leftColor = TFT_GREEN;
    middleColor = TFT_RED;
    rightColor = TFT_BLUE;
#else
    color_left = color_middle = color_right = TFT_BG;
#endif

    // Special symbols that draw manualy
    bool drawTopRightSquare = text == String("◱");
    bool drawBottomLeftSquare = text == String("◳");

    if (drawTopRightSquare || drawBottomLeftSquare) {
      sprite.createSprite(px_height_font, px_height_font);
      sprite.fillSprite(color_middle);
      int px_sqaure_size = (px_height_font / 2) - 6;
      if (drawTopRightSquare) {
        // down left
        sprite.fillRect(px_height_font / 2, 0, px_sqaure_size, px_sqaure_size,
                        TFT_YELLOW);
        // up right
        sprite.fillRect(0, px_height_font / 2, px_sqaure_size, px_sqaure_size,
                        TFT_YELLOW);
      } else if (drawBottomLeftSquare) {
        // left up
        sprite.fillRect(0, 0, px_sqaure_size, px_sqaure_size, TFT_YELLOW);
        // down right
        sprite.fillRect(px_height_font / 2, px_height_font / 2, px_sqaure_size,
                        px_sqaure_size, TFT_YELLOW);
      }

    } else {
      sprite.createSprite(px_width_sprite, px_height_font);
      sprite.fillSprite(color_middle);
      sprite.setTextColor(TFT_TEXT);
      sprite.setFreeFont(p_font);
      sprite.drawString(text, 0, 0);
    }
    // if (px_width_sprite < px_max_width) {
    //  Create and display left background sprite
    bg_left_sprite.createSprite(px_max_width - px_width_sprite, px_height_font);
    bg_left_sprite.fillSprite(color_left);
    int bg_left_x_cord = x - (px_max_width - px_width_sprite);
    int bg_left_y_cord = y + dy + (px_height_full * idx_row);
    bg_left_sprite.pushSprite(bg_left_x_cord, bg_left_y_cord);
    bg_left_sprite.deleteSprite();

    // Create and display right background sprite
    const int bg_right_x = x + px_width_sprite;
    const int bg_right_width = px_max_width - px_width_sprite;
    bg_right_sprite.createSprite(bg_right_width, px_height_font);
    bg_right_sprite.fillSprite(color_right);
    bg_right_sprite.pushSprite(bg_right_x, y + dy + (px_height_full * idx_row));
    bg_right_sprite.deleteSprite();
    //}

    // Create and display text sprite
    /*sprite.createSprite(px_width_sprite, px_height_font);
            sprite.fillSprite(color_middle);
            sprite.setTextColor(TFT_TEXT);
            sprite.setFreeFont(p_font);
            sprite.drawString(text, 0, 0);*/
    sprite.pushSprite(x, y + dy + (px_height_full * idx_row));
    sprite.deleteSprite();
  }


  /**
     * @brief Draws separator lines between rows on the screen.
     */
  void drawLines() const {
    // Create a TFT_eSprite for drawing separator lines
    TFT_eSprite lines_separator(&tft);
    // Create a N-pixel tall sprite
    lines_separator.createSprite(tft.width(), px_separate_line_height);

    for (int i = 1; i < cnt_rows; ++i) {
      int px_height_and_sprite = (tft.height() - (px_separate_line_height * i));
      int y_cord = px_height_and_sprite / cnt_rows * i;
      // Clear the sprite and draw a horizontal line
      lines_separator.fillSprite(TFT_BLACK);
      lines_separator.drawFastHLine(0, 0, tft.width(), TFT_BLACK);

      // Push the sprite to display at the specified Y-coordinate
      lines_separator.pushSprite(0, y_cord);
    }
    // Delete the sprite to free up memory
    lines_separator.deleteSprite();
  }


  /**
     * @brief Sets the maximum px_width of the name text in pixels.
     *
     * @param px_w Width of the name text in pixels to compare with the current
     * maximum.
     */
  void SetMaxNameTextWidth_px(int px_w) {
    if (px_w > px_max_width_name_text) {
      px_max_width_name_text = px_w;
      tft.fillScreen(TFT_BG);
    }
  }


  /**
     * @brief Sets the maximum px_width of the countdown text in pixels.
     *
     * @param px_w The px_width of the countdown text in pixels to compare with
     * the current maximum.
     */
  void SetMaxCountdownTextWidth_px(int px_w) {
    if (px_w > px_max_width_countdown_text) {
      px_max_width_countdown_text = px_w;
      tft.fillScreen(TFT_BG);
    }
  }


  /**
     * @brief Gets the maximum px_width of the countdown text in pixels.
     *
     * @return The maximum px_width of the countdown text in pixels.
     */
  int GetMaxCountdownTextWidth_px() const {
    return px_max_width_countdown_text;
  }


  /**
     * @brief Gets the maximum px_width of the name text in pixels.
     *
     * @return The maximum px_width of the name text in pixels.
     */
  int GetMaxNameTextWidth_px() const {
    return px_max_width_name_text;
  }


  /**
     * @brief Calculates the coordinate position of a line based on the specified
     * parameters.
     *
     * @param bodyLength The total length of the body where lines are positioned.
     * @param segmentMargin The margin between segments.
     * @param segmentLength The length of each segment.
     * @param segmentCount The total number of segments.
     * @param n The index of the line.
     *
     * @return The Coordinate position of the line.
     */
  int CalculateLineDistance_px(double bodyLength, double segmentMargin,
                               double segmentLength, int segmentCount,
                               int n) const {
    // Calculate the length occupied by segments
    double segmentsLength = segmentCount * segmentLength;

    // Calculate the length occupied by margins
    double marginsLength = (segmentCount - 1) * segmentMargin;

    // Add the lengths to get the total
    double totalSegmentsLength = segmentsLength + marginsLength;

    // Calculate the distance from the beginning of the body to the first line
    double x_firstLine = (bodyLength - totalSegmentsLength) / 2;

    // Calculate the final position based on the first line and segment spacing
    double finalPosition = x_firstLine + (n * (segmentLength + segmentMargin));

    // Convert the final position to an integer and return it
    return static_cast<int>(finalPosition);
  }


  /**
     * @brief Calculates the px_width of a text string using the specified p_font.
     *
     * @param p_font The p_font used for rendering the text.
     * @param str The text string to calculate the px_width for.
     *
     * @return The px_width of the text string in pixels.
     */
  int CalculateFontWidth_px(const GFXfont* p_font, const String& str) const {
    // This symbol not exist in p_font this squares draw manuany
    // In squeres Width == Height
    if (str == String("◱") || str == String("◳")) {
      return CalculatefontHeight_px(p_font);  // height == px_width in square
    }

    TFT_eSprite Calculator = TFT_eSprite(&tft);
    Calculator.setFreeFont(p_font);
    int px_w = Calculator.textWidth(str);
    Calculator.deleteSprite();
    return px_w;
  }


  /**
     * @brief Calculates the height of the text when rendered using the specified
     * p_font.
     *
     * This function creates a temporary sprite, sets the provided p_font, and
     * then retrieves the p_font height.
     *
     * @param p_font The p_font used for rendering the text.
     *
     * @return The height of the text when rendered with the specified p_font in
     * pixels.
     */
  int CalculatefontHeight_px(const GFXfont* p_font) const {
    // Getter V1
    // TFT_eSprite Calculator = TFT_eSprite(&tft);
    // Calculator.createSprite(tft.px_width() * 8, tft.height());
    // Calculator.setFreeFont(p_font);
    // return Calculator.px_height_font();

    // Getter V2
    // Check if the height has already been calculated for this p_font
    auto it = font_height_cache.find(p_font);
    if (it != font_height_cache.end()) {
      //   If yes, return the stored value
      return it->second;
    }

    const char* t =
      "abcdefghijklmnopqrstuvwxyz"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "0123456789";
    TFT_eSprite Calculator = TFT_eSprite(&tft);
    Calculator.setFreeFont(p_font);

    uint16_t len = strlen(t);
    uint16_t maxH = 0;

    for (uint16_t i = 0; i < len; i++) {
      uint16_t unicode = t[i];
      uint16_t gNum = unicode - p_font->first;
      if (gNum >= p_font->last) continue;  // Skip undefined characters

      GFXglyph* glyph = &(((GFXglyph*)p_font->glyph))[gNum];
      uint16_t h = glyph->height;

      if (h > maxH) maxH = h;
    }

    int height = maxH ? maxH + 1 : 0;

    // Save the calculated value in the cache
    font_height_cache[p_font] = height;

    return height;
  }


  void SetMinTextSprite_px(int px_min) {
    // Serial.println(px_min_text_sprite);
    px_min_text_sprite = min(px_min_text_sprite, px_min);
  }


  ///< The maximum px_width of the name text in pixels.
  int px_max_width_name_text;

  ///< The maximum px_width of the countdown text in pixels.
  int px_max_width_countdown_text;

  ///< The px_width bewwen tables.
  int px_separate_line_height;

  ///< The px_width bewwen tables.
  int px_margin_lines;

  ///< The number of rows on the screen.
  int cnt_rows;

  ///< The number of lines of text that can be displayed in each idx_row.
  int cnt_lines_in_row;

  ///< The margin value for text and components.
  const int px_margin;

  ///< Coordinates for text scrolling in each idx_row.
  std::vector<std::vector<int>> vec_scrolls_coords;
  std::vector<std::vector<bool>> vec_init_scrolls_coords;

  int px_min_text_sprite;
  mutable std::map<const GFXfont*, int> font_height_cache;

public:
  void FullResetScroll() {
    for (size_t i = 0; i < vec_scrolls_coords.size(); ++i) {
      for (size_t j = 0; j < vec_scrolls_coords[i].size(); ++j) {
        vec_scrolls_coords[i][j] = 0;
      }
    }

    for (size_t i = 0; i < vec_init_scrolls_coords.size(); ++i) {
      for (size_t j = 0; j < vec_init_scrolls_coords[i].size(); ++j) {
        vec_init_scrolls_coords[i][j] = false;
      }
    }
  }


  void SelectiveResetScroll(const std::vector<bool>& isNeedReset) {
    // printf("\n\n%d %d\n\n ", vec_init_scrolls_coords.size(),
    // isNeedReset.size());
    for (size_t i = 0; i < vec_scrolls_coords.size(); ++i) {
      for (size_t j = 0; j < vec_scrolls_coords[i].size(); ++j) {
        if (isNeedReset[i]) {
          vec_scrolls_coords[i][j] = 0;
        }
      }
    }

    for (size_t i = 0; i < vec_init_scrolls_coords.size(); ++i) {
      // printf("%d ", isNeedReset[i] ? 1 : 0);
      for (size_t j = 0; j < vec_init_scrolls_coords[i].size(); ++j) {
        if (isNeedReset[i]) {
          vec_init_scrolls_coords[i][j] = false;
        }
      }
    }
    // printf("\n\n");
  }
};



class TraficClock {
public:
  TraficClock(long ms_perCountdown, long cd_perIterations, long it_perHour)
    : kMillisecondsPerCountdown(ms_perCountdown),
      kCountdownsPerIteration(cd_perIterations),
      kIterationsPerHour(it_perHour) {
    Reset();
  }

private:
  const unsigned long kMillisecondsPerCountdown = 5000;
  const long kCountdownsPerIteration = 2;
  const long kIterationsPerHour = 2;
  unsigned long start_time_;


  unsigned long Milliseconds() const {
    return millis();
  }


  long GetTotalCountdown() const {
    unsigned long totalMilliseconds = Milliseconds();
    return totalMilliseconds / kMillisecondsPerCountdown;
  }


  long GetTotalIteration() const {
    return GetTotalCountdown() / kCountdownsPerIteration;
  }


public:
  void Reset() {
    start_time_ = millis();
  }


  long GetCountdown() const {
    long seconds = GetTotalCountdown() % kCountdownsPerIteration;
    return seconds;
  }


  long GetIteration() const {
    long iterations = GetTotalIteration() % kIterationsPerHour;
    return iterations;
  }


  long GetFullCycle() const {
    long cycle = GetTotalIteration() / kIterationsPerHour;
    return cycle;
  }


  void PrintTime() const {
#if 1
    Serial.print("Time: ");
    Serial.print(GetFullCycle());
    Serial.print(":");
    Serial.print(GetIteration());
    Serial.print(":");
    Serial.print(GetCountdown());
    Serial.print(".");
    Serial.println(Milliseconds());
#endif
  }
};

class TraficManager {
public:
  TraficManager()
    : shift_cnt(0), coundown_idx(0), p_trafic_clock(nullptr),
      last_toggle_time(0), show_first_departure(true) {}


  ~TraficManager() {
    delete p_trafic_clock;
  }


  // Block 1: Update Traffic Data
  void update(const std::vector<Monitor>& mainData, const std::vector<Monitor>& line3Data = std::vector<Monitor>()) {
    //shift_cnt = 0;
    prev_iterations = 0;
    //SmartWatch sm(__FUNCTION__);

    // Organize monitors by display line
    const Config& cfg = global_settings.GetConfig();

    // If we have separate Line 3 data, handle it specially
    if (cfg.GetLinesCountAsInt() == 3 && !line3Data.empty()) {
      monitors_per_line = GetMonitorsPerDisplayLineWithSeparateLine3(mainData, line3Data, cfg);
    } else {
      monitors_per_line = GetMonitorsPerDisplayLine(mainData, cfg);
    }

    // Sort each line's monitors by countdown
    for (auto& line_monitors : monitors_per_line) {
      sortTrafic(line_monitors);
    }

    if (p_trafic_clock) {
      p_trafic_clock->Reset();
#ifdef DEBUG_SERIAL_WIEN_MONITOR
      Serial.println("updater_");
#endif
      p_screen->FullResetScroll();
    }

    // Reset toggle timer on data update
    last_toggle_time = millis();
    show_first_departure = true;
  }


  /**
   * @brief Updates the screen with traffic information.
   */
  void updateScreen() {
    // If no data yet, show loading message or empty screen with structure
    if (monitors_per_line.empty()) {
      // Initialize with empty structure based on config
      const Config& cfg = global_settings.GetConfig();
      int numLines = cfg.GetLinesCountAsInt();
      monitors_per_line.resize(numLines);

      // Draw empty screen with proper structure
      DrawTraficOnScreen();
      return;
    }

    // Check if 4 seconds have passed to toggle between departures
    unsigned long current_time = millis();
    if (current_time - last_toggle_time >= 4000) {
      show_first_departure = !show_first_departure;
      last_toggle_time = current_time;
      p_screen->FullResetScroll();  // Reset scroll when toggling
    }

    DrawTraficOnScreen();
  }


  /**
   * @brief Resets the scrolling for monitors that have different descriptions.
   * @param currentTrafficSubset The current subset of traffic monitors.
   * @param futureSubset The future subset of traffic monitors.
   */
  void SelectiveReset(const std::vector<Monitor>& currentTraficSubset,
                      const std::vector<Monitor>& futureSubset) {
    // p_screen->PrintCordDebug();
    if (currentTraficSubset.size() == futureSubset.size()) {
      //PrintDegbugMonitors(currentTraficSubset);
      //PrintDegbugMonitors(futureSubset);

      size_t size = futureSubset.size();
      std::vector<bool> isNeedReset(size, true);
      for (size_t i = 0; i < size; ++i) {
        if (currentTraficSubset[i].description.isEmpty()
            || futureSubset[i].description.isEmpty()) {
          isNeedReset[i] = true;
        } else if (currentTraficSubset[i].description
                   == futureSubset[i].description) {
          isNeedReset[i] = false;
        }
      }

      // update only for difrent names
      if (currentTraficSubset.size() > 1) {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
        for (size_t k = 0; k < isNeedReset.size(); k++) {
          Serial.print(isNeedReset[k]);
          Serial.print(" ");
        }
        Serial.println("");
#endif
        p_screen->SelectiveResetScroll(isNeedReset);
      }
    }

    // p_screen->PrintCordDebug();
  }


  /**
   * @brief Sorts traffic monitors based on countdown values.
   * @param v The vector of traffic monitors to be sorted.
   */
  void sortTrafic(std::vector<Monitor>& v) {
    std::sort(v.begin(), v.end(), [](const Monitor& a, const Monitor& b) {
      return a.countdown < b.countdown;
    });
  }


  /**
   * @brief Returns a valid countdown string at the given index.
   * @param c The vector of countdown values.
   * @param index The index to retrieve the countdown value from.
   * @return A valid countdown string.
   */
  String GetValidCountdown(const std::vector<int>& c, size_t index) {
    if (c.empty()) {
      return String();
    }

    size_t best_index = std::min(index, c.size() - 1);
    return String(c[best_index], DEC);
  }


  /**
   * @brief Draws traffic information on the screen using per-line data.
   */
  void DrawTraficOnScreen() {
    size_t numLines = global_settings.GetConfig().GetLinesCountAsInt();

    std::vector<ScreenEntity> vec_screen_entity;

    // Iterate through each display line
    for (size_t display_line = 0; display_line < numLines; ++display_line) {
      ScreenEntity entity;
      std::vector<String> clean_str;
      for (size_t x = 0; x < global_settings.cnt_screen_lines_per_rows; ++x) {
        clean_str.push_back("");
        clean_str.push_back("");
      }

      // Check if this display line has any monitors
      if (display_line < monitors_per_line.size()
          && !monitors_per_line[display_line].empty()) {

        const auto& line_monitors = monitors_per_line[display_line];

        // Find the monitor with the earliest departure
        if (!line_monitors.empty()) {
          const Monitor& currentMonitor = line_monitors[0];

          // Determine which countdown to show (first or second) from the SAME monitor
          size_t countdown_idx = 0;
          if (!show_first_departure && currentMonitor.countdown.size() > 1) {
            countdown_idx = 1;  // Show second countdown from same route
          }

          // Set the line name (right side)
          entity.right_txt = currentMonitor.name;

          // Set the countdown (left side) - use the countdown at countdown_idx
          if (countdown_idx < currentMonitor.countdown.size()) {
            if (currentMonitor.countdown[countdown_idx] == 0) {
              // Blink for imminent departure
              if ((millis() / 1000) % 2) {
                entity.left_txt = String("◱");
              } else {
                entity.left_txt = String("◳");
              }
            } else {
              entity.left_txt = String(currentMonitor.countdown[countdown_idx], DEC);
            }
          } else {
            entity.left_txt = "";
          }

          // Set the destination text (middle)
          auto splitted_string = getSplittedStringFromCache(currentMonitor.towards);
          size_t cnt_sub_rows = min(
            static_cast<size_t>(global_settings.cnt_screen_lines_per_rows),
            splitted_string.size());
          size_t cnt_char_splitted_text = 0;
          for (size_t j = 0; j < cnt_sub_rows; ++j) {
            clean_str[j] = splitted_string[j];
            cnt_char_splitted_text += splitted_string[j].length();
          }

          if (!currentMonitor.towards.isEmpty()) {
            if ((static_cast<float>(cnt_char_splitted_text)
                   / static_cast<float>(currentMonitor.towards.length())
                 < 0.5)) {
              String str_max = getMaximumPosibleSingleNoScrollWord(
                currentMonitor.towards);
              clean_str[0] = str_max;
              clean_str[1].clear();
            }
          }

          // Add description if present
          if (currentMonitor.description.length()) {
            clean_str[1] = currentMonitor.description;
          }
        }
      } else {
        // No departures for this line - show configured line name if any
        const Config& cfg = global_settings.GetConfig();
        String configuredLine = cfg.GetLineName(display_line + 1);
        if (!configuredLine.isEmpty()) {
          entity.right_txt = configuredLine;
          entity.left_txt = "-";
          // Show "No departures" or similar message
        } else {
          entity.right_txt = "";
          entity.left_txt = "";
        }
      }

      entity.lines = clean_str;
      vec_screen_entity.push_back(entity);
    }

    // Render all entities
    for (size_t i = 0; i < vec_screen_entity.size(); ++i) {
      p_screen->SetRow(vec_screen_entity[i], i);
    }
  }

  int last_min_size = -1;  // Initialize last_min_size to an invalid value
  std::map<String, std::vector<String>> SplittedStringCache;

  /**
   * @brief Retrieves or generates a vector of split strings from a cache.
   * @param key_string The key string used to retrieve or generate split strings.
   * @return A vector of split strings either from the cache or generated.
   */
  std::vector<String> getSplittedStringFromCache(const String& key_string) {
    int min_size = p_screen->GetMinTextSprite_px();

    if (last_min_size != min_size) {
      // Clear the cache if min_size has changed
      SplittedStringCache.clear();
      last_min_size = min_size;
    }

    auto cacheIt = SplittedStringCache.find(key_string);
    if (cacheIt != SplittedStringCache.end()) {
      // Found in the cache, return it
      //Serial.println("return from cache");
      return cacheIt->second;
    }

    // Generate and cache the result
    std::vector<String> generatedStrings = splitToString(key_string);
    SplittedStringCache[key_string] = generatedStrings;
    return generatedStrings;
  }
  /**
 * Extracts the maximum substring from 'str' that fits the display.
 *
 * @param str The input string to process.
 * @return The maximum substring that fits, or an empty string if too short.
 */
  String getMaximumPosibleSingleNoScrollWord(const String& str) {
    String currentWord = "";
    size_t strLen = str.length();
    // Preallocate memory to prevent dynamic resizing
    currentWord.reserve(strLen);

    for (size_t i = 0; i < strLen; i++) {
      if (p_screen->IsEnoughSpaceForMiddleText(str.substring(0, i))) {
        currentWord += str[i];
      } else {
        break;
      }
    }

    if (currentWord.length() >= 2) {
      // Remove the last character
      currentWord.remove(currentWord.length() - 2);
    } else {
      currentWord = "";  // Handle the case when the string is too short
    }

    return currentWord;
  }

  /**
     * @brief Splits a string into individual words and stores them in a vector.
     * @param str The input string to be split.
     * @param words A vector to store the individual words.
     */
  void splitStringToWords(const String& str, std::vector<String>& words) {
    // SmartWatch sm(__FUNCTION__);
    String currentWord = "";
    words.reserve(32);
    for (size_t i = 0; i < str.length(); i++) {
      currentWord += str[i];

      // If the current character is a space or a hyphen
      if (str[i] == ' ' || str[i] == '-') {
        // Add the current word to the words vector
        if (p_screen->IsEnoughSpaceForMiddleText(currentWord)) {
          words.push_back(currentWord);
        }
        currentWord = "";
      }
    }

    // Add the remaining word if it's not empty
    if (!currentWord.isEmpty()
        && p_screen->IsEnoughSpaceForMiddleText(currentWord)) {
      words.push_back(currentWord);
    }
  }

  /**
     * @brief Splits a string into subtexts that fit within one line.
     * @param input The input string to be split into subtexts.
     * @return A vector of subtexts that fit within one line.
     */
  std::vector<String> splitToString(const String& input) {
    // SmartWatch sm(__FUNCTION__);
    std::vector<String> words;
    splitStringToWords(input, words);

    // Create a list of subtexts
    std::vector<String> subtexts;
    String currentSubtext = "";

    // Add words to the subtexts list as long as they fit in one line
    for (size_t i = 0; i < words.size(); i++) {
      if (p_screen->IsEnoughSpaceForMiddleText(currentSubtext + " " + words[i])) {
        currentSubtext += words[i];
      } else {
        if (!currentSubtext.isEmpty()) {
          subtexts.push_back(currentSubtext);
        }
        currentSubtext = words[i];
      }
    }

    if (!currentSubtext.isEmpty()) {
      subtexts.push_back(currentSubtext);
    }

    for (size_t i = 0; i < subtexts.size(); i++) {
      if (!subtexts[i].isEmpty() && (subtexts.size() - 1 != i)) {
        subtexts[i] = subtexts[i].substring(0, subtexts[i].length() - 1);
      }
    }

    return subtexts;
  }

private:
  std::vector<std::vector<Monitor>> monitors_per_line;  // Monitors organized by display line
  int shift_cnt;
  int coundown_idx;
  TraficClock* p_trafic_clock;
  long prev_iterations = 0;
  unsigned long last_toggle_time;  // Time of last departure toggle
  bool show_first_departure;       // Toggle between first and second departure
};

////////////////////////////Functions////////////////////////////////////////////


void PrintDebugMonitor(const Monitor& t) {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
  Serial.println("==========");
  Serial.println("Name: " + t.name);

  // tft.setAddrWindow(10, 20, 100, 80);

  Serial.println("Description: " + t.description);
  Serial.print("Countdown: ");
  for (auto& c : t.countdown) {
    Serial.print(c);
    Serial.print(", ");
  }
  Serial.println();
  Serial.println("==========");
#endif
}


String FixJsonMistake(String word) {
  word = Screen::ConvertGermanToLatin(word);
  // Check if the input string contains spaces
#if DEBUG_SERIAL_WIEN_MONITOR
  Serial.println(word);
#endif
  word.trim();
  if (word.indexOf('-') == -1
      && word.indexOf(' ') == -1
      && word.length() > 1) {
    for (int i = 0; i < word.length(); i++) {
      word[i] = tolower(word[i]);  // Convert all letters to lowercase
    }
    word[0] = toupper(word[0]);  // Convert the first letter to uppercase
  }
#if DEBUG_SERIAL_WIEN_MONITOR
  Serial.println(word);
#endif
  return word;
}


void PrintDegbugMonitors(const std::vector<Monitor>& monitors) {
  for (const auto& monitor : monitors) {
    PrintDegbugMonitors(monitor);
  }
}


std::vector<Monitor> GetMonitorsFromJson(const String& json) {
  //SmartWatch sm(__FUNCTION__);
  std::vector<Monitor> monitors_vec;

  DynamicJsonDocument root(2048 * 8);
  DeserializationError error =
    deserializeJson(root, json, DeserializationOption::NestingLimit(64));

  if (error) {
#ifdef DEBUG_SERIAL_WIEN_MONITOR
    Serial.println("Failed to parse JSON response.");
    Serial.println(error.c_str());
#endif
    // screenInit();  // draw empty screen
    delay(global_settings.ms_error_resset_delay);

    return monitors_vec;
  }

  const JsonObject data = root["data"];
  const JsonArray traffic_infos = data["trafficInfos"];
  std::map<String, String> lines_to_description_map;
  for (const auto& traffic_info : traffic_infos) {
    if (traffic_info.containsKey("description")) {
      const String description = Screen::ConvertGermanToLatin(
        traffic_info["description"].as<String>());
      const JsonArray related_lines = traffic_info["relatedLines"];

      for (const auto& related_line : related_lines) {
        String name_transport_with_descr = related_line.as<String>();

        lines_to_description_map[name_transport_with_descr] = description;
        Serial.print("descr");
        Serial.print(name_transport_with_descr);
        Serial.println(description);
      }
    }
  }

  const JsonArray monitors = data["monitors"];
  for (const auto& monitor : monitors) {
    const JsonArray lines = monitor["lines"];
    for (const auto& line : lines) {
      const JsonObject departures = line["departures"];
      Monitor cur_monitor;

      const JsonVariant departure = departures["departure"];

      if (departure.is<JsonArray>()) {
        const JsonArray& departuresArray = departure.as<JsonArray>();
        // cur_monitor.countdown.reserve(departuresArray.size());

        for (const auto& departureItem : departuresArray) {
          Monitor cur_monitor;

          int countdown = -1;
          if (departureItem.containsKey("vehicle")
              && departureItem.containsKey("departureTime")) {
            // Serial.println("_____2");

            const JsonObject vehicle = departureItem["vehicle"];
            cur_monitor.name = vehicle["name"].as<String>();
            cur_monitor.towards =
              FixJsonMistake(vehicle["towards"].as<String>());

            const JsonObject departureTime = departureItem["departureTime"];
            countdown = departureTime["countdown"].as<int>();
            countdown = departureTime["countdown"].as<int>();
            if (countdown != -1) {
              cur_monitor.countdown.push_back(countdown);
            }
            //serializeJsonPretty(departureItem, Serial);

          } else if (departureItem.containsKey("departureTime")) {
            // Serial.println("_____1");
            // teake information in upper lavel
            cur_monitor.name = line["name"].as<String>();
            cur_monitor.towards = FixJsonMistake(line["towards"].as<String>());

            const JsonObject departureTime = departureItem["departureTime"];
            countdown = departureTime["countdown"].as<int>();
            if (countdown != -1) {
              cur_monitor.countdown.push_back(countdown);
            }

            // serializeJsonPretty(departureItem, Serial);
          }
          //////////////////////////
          // description
          cur_monitor.description = lines_to_description_map[cur_monitor.name];
          /////////////////////////
          auto it =
            std::find_if(monitors_vec.begin(), monitors_vec.end(),
                         [&cur_monitor](const Monitor& monitor) {
                           return monitor.name == cur_monitor.name
                                  && monitor.towards == cur_monitor.towards;
                         });

          if (it == monitors_vec.end()) {
            monitors_vec.push_back(
              cur_monitor);  // If not found, add a new Monitor
          } else {
            // If found, update the existing Monitor with the countdown counter
            if (countdown != -1) {
              it->countdown.push_back(countdown);
            }
          }
        }
      }
    }
  }

  return monitors_vec;
}


String GetRandomString(int maxLength) {
  // Define valid characters
  String validChars =
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

  // Get a random string length from 1 to maxLength
  int length = random(1, maxLength + 1);

  // Generate a random string
  String randomString = "";
  for (int i = 0; i < length; i++) {
    int randomIndex = random(validChars.length());
    randomString += validChars[randomIndex];
  }

  return randomString;
}


/**
 * @brief Splits a string by a specified separator character.
 * @param data The input string to be split.
 * @param separator The character used for splitting.
 * @return A vector of substrings resulting from the split operation.
 */
std::vector<String> GetSplittedStrings(String data, char separator) {
  int separatorIndex = 0;
  std::vector<String> result;

  while (separatorIndex != -1) {
    separatorIndex = data.indexOf(separator);
    String chunk = data.substring(0, separatorIndex);
    result.push_back(chunk);
    if (separatorIndex != -1) {
      data = data.substring(separatorIndex + 1);
    }
  }

  return result;
}


template<typename T>
std::vector<T> cyclicSubset(const std::vector<T>& input,
                            size_t N,
                            size_t start) {
  std::vector<T> result;
  size_t size = input.size();

  // If the input vector is empty or N is 0, return an empty result.
  if (input.empty() || N == 0) {
    return result;
  }

  // Let's start with the start element and add elements to the result.
  for (size_t i = start; i < start + N; ++i) {
    result.push_back(input[i % size]);
  }

  return result;
}


/**
 * @brief Filters a vector of Monitor objects based on a filter string.
 * @param data The input vector of Monitor objects.
 * @param filter The filter string to match against Monitor names.
 * @return A vector containing Monitor objects that match the filter.
 */
std::vector<Monitor> GetFilteredMonitors(const std::vector<Monitor>& data,
                                         const String& filter) {
  if (filter.isEmpty()) {
    return data;
  }
  std::vector<String> custom_names = GetSplittedStrings(filter, ',');
  std::vector<Monitor> result;

  for (auto& monitor : data) {
    auto it = std::find(custom_names.begin(), custom_names.end(), monitor.name);
    if (it != custom_names.end()) {
      result.push_back(monitor);
    }
  }

  return result;
}


/**
 * @brief Organizes monitors into per-display-line vectors based on config.
 * @param data The input vector of all Monitor objects.
 * @param config The configuration containing line assignments.
 * @return A vector of vectors, where each inner vector contains monitors for that display line.
 */
std::vector<std::vector<Monitor>> GetMonitorsPerDisplayLine(
  const std::vector<Monitor>& data,
  const Config& config) {

  int numLines = config.GetLinesCountAsInt();
  std::vector<std::vector<Monitor>> result(numLines);

  for (int i = 0; i < numLines; i++) {
    String lineName = config.GetLineName(i + 1);

    if (lineName.isEmpty()) {
      // If no line specified, this display line can show any departure
      result[i] = data;
    } else {
      // Filter to show only departures from this specific line
      for (const auto& monitor : data) {
        if (monitor.name == lineName) {
          result[i].push_back(monitor);
        }
      }
    }
  }

  return result;
}


/**
 * @brief Organizes monitors into per-display-line vectors with separate data for line 3.
 * @param mainData The input vector of Monitor objects for lines 1 and 2.
 * @param line3Data The input vector of Monitor objects specifically for line 3.
 * @param config The configuration containing line assignments.
 * @return A vector of vectors, where each inner vector contains monitors for that display line.
 */
std::vector<std::vector<Monitor>> GetMonitorsPerDisplayLineWithSeparateLine3(
  const std::vector<Monitor>& mainData,
  const std::vector<Monitor>& line3Data,
  const Config& config) {

  int numLines = config.GetLinesCountAsInt();
  std::vector<std::vector<Monitor>> result(numLines);

  // Process lines 1 and 2 from main data
  for (int i = 0; i < 2 && i < numLines; i++) {
    String lineName = config.GetLineName(i + 1);

    if (lineName.isEmpty()) {
      // If no line specified, this display line can show any departure
      result[i] = mainData;
    } else {
      // Filter to show only departures from this specific line
      for (const auto& monitor : mainData) {
        if (monitor.name == lineName) {
          result[i].push_back(monitor);
        }
      }
    }
  }

  // Process line 3 from separate data source
  if (numLines >= 3) {
    String lineName = config.GetLineName(3);

    if (lineName.isEmpty()) {
      // If no line specified, show all from line 3 data
      result[2] = line3Data;
    } else {
      // Filter line 3 data to show only this specific line
      for (const auto& monitor : line3Data) {
        if (monitor.name == lineName) {
          result[2].push_back(monitor);
        }
      }
    }
  }

  return result;
}


void PrintDegbugMonitors(const Monitor& monitor) {
#if 1
  Serial.print("Name: ");
  Serial.print(monitor.name);
  // Serial.print(" Towards: ");
  // Serial.print(monitor.towards);
  Serial.print(" Description: ");
  Serial.print(monitor.description);

  /*Serial.println("Countdown:");
    for (int i = 0; i < monitor.countdown.size(); i++) {
      Serial.print("  ");
      Serial.print(monitor.countdown[i]);
    }*/
  Serial.println();
#endif
}


/**
 * @brief Updates data for a specific task.
 * @param pvParameters Pointer to task-specific parameters.
 */
void UpdateDataTask(void* pvParameters) {
  std::vector<Monitor> allTrafficSetInit;
  const int& delay_planned_ms = global_settings.ms_task_delay_data_update;
  int delay_real_ms = 0;

  // Retry delays in milliseconds: 5s, 10s, 15s, 20s, 40s
  const int retry_delays_ms[] = {5000, 10000, 15000, 20000, 40000};
  const int num_retries = sizeof(retry_delays_ms) / sizeof(retry_delays_ms[0]);

  while (true) {
    bool update_successful = false;
    int retry_attempt = 0;

    // Keep trying until we succeed or exhaust all retries
    while (!update_successful && retry_attempt <= num_retries) {
      {  //SmartWatch start
        SmartWatch sm(__FUNCTION__);
        // Fetch JSON data (may take several seconds)
        const auto& cfg = global_settings.GetConfig();

        if (retry_attempt == 0) {
          // Normal update - print debug info
          Serial.print("Main RBL: ");
          Serial.println(cfg.GetLinesRblAsString());
          Serial.print("Line 1 config: ");
          Serial.println(cfg.GetLineName(1));
          Serial.print("Line 2 config: ");
          Serial.println(cfg.GetLineName(2));
          Serial.print("Line 3 config: ");
          Serial.println(cfg.GetLineName(3));
        } else {
          Serial.print("Retry attempt ");
          Serial.print(retry_attempt);
          Serial.print(" of ");
          Serial.println(num_retries);
        }

        // Fetch main RBL data using cache manager
        const String& main_rbl = cfg.GetLinesRblAsString();
        const String& rbl_json = api_cache.FetchRBL(main_rbl);
        std::vector<Monitor> tempTrafficSet = GetMonitorsFromJson(rbl_json);
        Serial.print("Monitors fetched from main RBL: ");
        Serial.println(tempTrafficSet.size());

        const String& raw_filter = cfg.GetLinesFilterAsString();
        tempTrafficSet = GetFilteredMonitors(tempTrafficSet, raw_filter);
        Serial.print("Monitors after filter: ");
        Serial.println(tempTrafficSet.size());

        // Check if we need to fetch separate data for Line 3
        std::vector<Monitor> line3TrafficSet;
        if (cfg.GetLinesCountAsInt() == 3 && !cfg.GetLine3RBL().isEmpty()) {
          const String& line3_rbl = cfg.GetLine3RBL();

          // Check if Line 3 uses the same RBL as main
          if (line3_rbl == main_rbl) {
            Serial.println("Line 3 uses same RBL as main - reusing data");
            line3TrafficSet = tempTrafficSet; // Reuse already fetched data
          } else {
            Serial.print("Fetching separate RBL for Line 3: ");
            Serial.println(line3_rbl);

            // Use cache manager with automatic delay
            const String& line3_rbl_json = api_cache.FetchRBL(line3_rbl);
            line3TrafficSet = GetMonitorsFromJson(line3_rbl_json);
            Serial.print("Line 3 monitors fetched: ");
            Serial.println(line3TrafficSet.size());
          }

          // Apply Line 3 specific filter if configured, otherwise use main filter
          String line3_filter = cfg.GetLine3Filter();
          if (line3_filter.isEmpty()) {
            line3_filter = raw_filter;
          }
          line3TrafficSet = GetFilteredMonitors(line3TrafficSet, line3_filter);
          Serial.print("Line 3 monitors after filter: ");
          Serial.println(line3TrafficSet.size());
        }

        // Acquire the data mutex
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          if (!tempTrafficSet.empty() || !line3TrafficSet.empty()) {
            // Update succeeded - save the new data
            allTrafficSetInit = tempTrafficSet;

            // Pass both datasets to the traffic manager
            pTraficManager->update(allTrafficSetInit, line3TrafficSet);
#ifdef DEBUG_SERIAL_WIEN_MONITOR
            Serial.println("Monitor data updated.");
#endif
            update_successful = true;
          } else {
            // Update failed - keep old data
            Serial.println("Update failed - keeping old data visible");
          }
          // Release the data mutex
          xSemaphoreGive(dataMutex);
        }

        int execution_ms = static_cast<int>(sm.GetExecution_ms());

        if (!update_successful) {
          // Failed to get data, schedule retry
          if (retry_attempt < num_retries) {
            int retry_delay = retry_delays_ms[retry_attempt];
            Serial.print("Will retry in ");
            Serial.print(retry_delay / 1000);
            Serial.println(" seconds...");
            delay_real_ms = max(0, retry_delay - execution_ms);
          } else {
            // Exhausted all retries, go back to normal schedule
            Serial.println("All retries exhausted, resuming normal schedule");
            delay_real_ms = max(0, delay_planned_ms - execution_ms);
          }
        } else {
          // Success on first try or after retry, use normal delay
          delay_real_ms = max(0, delay_planned_ms - execution_ms);
        }
      }  //SmartWatch end

      vTaskDelay(pdMS_TO_TICKS(delay_real_ms));
      retry_attempt++;
    }
  }
}


/**
 * @brief Updates the screen for a specific task.
 * @param pvParameters Pointer to task-specific parameters.
 */
void ScreenUpdateTask(void* pvParameters) {
  const int delay_ms = global_settings.ms_task_delay_screen_update;

  while (true) {
    // Acquire the data mutex
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      pTraficManager->updateScreen();
      // Serial.println("Screen updated.");

      // Release the data mutex
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(delay_ms));
  }
}


/**
 * @brief Retrieves JSON data from a specified URL.
 * @param rbl_id The resource identifier to fetch data from.
 * @return The JSON data as a string or an empty string if unsuccessful.
 */
String GetJson(const String& rbl_id) {
  //SmartWatch sm(__FUNCTION__);

  if (WiFi.status() != WL_CONNECTED) {
    return "";
  }

  HTTPClient http;
  String url = URL_BASE + rbl_id;

  // Print the URL to the serial port
  Serial.println(url);

  // Start the HTTP request
  http.begin(url);

  // Get the status code of the response
  int httpCode = http.GET();

  // Print the status code to the serial port
  Serial.print("HTTP status code: ");
  Serial.println(httpCode);

  // Check if the request was successful
  if (httpCode <= 0) {
    // The request failed
    Serial.println("Error on HTTP request");
  } else if (httpCode == HTTP_CODE_OK) {
    // The request was successful
    // Get the response payload as a string
    String payload = http.getString();
    http.end();
    return payload;
  }
  http.end();
  return "";
}


/**
 * @brief Manages Wi-Fi configuration using WiFiManager.
 */
void WiFiManagerTask() {
  //SmartWatch sm(__FUNCTION__);
  WiFiManager wifiManager;
  wifiManager.setDebugOutput(true);
  //const char* custom_html = "<style>button{background-color:red;}</style>";
  //wifiManager.setCustomHeadElement(custom_html);

  // Load saved configuration
  const Config& old_config = global_settings.GetConfig();
  Serial.println("getter");
  Config new_config;
  // Convert configuration values to strings
  const String& rblValue = old_config.GetLinesRblAsString();
  const String& countValue = old_config.GetLinesCountAsString();
  const String& filterValue = old_config.GetLinesFilterAsString();
  const String& line1Value = old_config.GetLineName(1);
  const String& line2Value = old_config.GetLineName(2);
  const String& line3Value = old_config.GetLineName(3);
  const String& line3RblValue = old_config.GetLine3RBL();
  const String& line3FilterValue = old_config.GetLine3Filter();

  // Create Wi-FiManager parameters
  WiFiManagerParameter custom_html_elem_hr("<hr>");
  String cpp_lines_rbl_promt = StringDatabase::GetRBLPrompt();

  String cpp_line_count_promt = StringDatabase::GetLineCountPrompt(
    Config::cnt_min_lines, Config::cnt_max_lines, Config::cnt_default_lines);

  String cpp_line_filter_promt = StringDatabase::GetLineFilterPrompt();
  String cpp_line1_prompt = StringDatabase::GetLine1Prompt();
  String cpp_line2_prompt = StringDatabase::GetLine2Prompt();
  String cpp_line3_prompt = StringDatabase::GetLine3Prompt();
  String cpp_line3_rbl_prompt = StringDatabase::GetLine3RBLPrompt();
  String cpp_line3_filter_prompt = StringDatabase::GetLine3FilterPrompt();
  String cpp_ssid = StringDatabase::GetWiFissid();
  String cpp_instruction = StringDatabase::GetInstructionsText();
  WiFiManagerParameter rblParam("lines_rbl", cpp_lines_rbl_promt.c_str(),
                                rblValue.c_str(), 64);

  WiFiManagerParameter countParam("lines_count", cpp_line_count_promt.c_str(),
                                  countValue.c_str(), 64);
  WiFiManagerParameter filterParam(
    "lines_filter", cpp_line_filter_promt.c_str(), filterValue.c_str(), 64);

  WiFiManagerParameter line1Param("line_1_name", cpp_line1_prompt.c_str(),
                                  line1Value.c_str(), 64);
  WiFiManagerParameter line2Param("line_2_name", cpp_line2_prompt.c_str(),
                                  line2Value.c_str(), 64);
  WiFiManagerParameter line3Param("line_3_name", cpp_line3_prompt.c_str(),
                                  line3Value.c_str(), 64);
  WiFiManagerParameter line3RblParam("line_3_rbl", cpp_line3_rbl_prompt.c_str(),
                                     line3RblValue.c_str(), 64);
  WiFiManagerParameter line3FilterParam("line_3_filter", cpp_line3_filter_prompt.c_str(),
                                        line3FilterValue.c_str(), 64);

  // Add parameters to Wi-FiManager
  wifiManager.addParameter(&rblParam);
  wifiManager.addParameter(&custom_html_elem_hr);
  wifiManager.addParameter(&countParam);
  wifiManager.addParameter(&custom_html_elem_hr);
  wifiManager.addParameter(&filterParam);
  wifiManager.addParameter(&custom_html_elem_hr);
  wifiManager.addParameter(&line1Param);
  wifiManager.addParameter(&custom_html_elem_hr);
  wifiManager.addParameter(&line2Param);
  wifiManager.addParameter(&custom_html_elem_hr);
  wifiManager.addParameter(&line3Param);
  wifiManager.addParameter(&custom_html_elem_hr);
  wifiManager.addParameter(&line3RblParam);
  wifiManager.addParameter(&custom_html_elem_hr);
  wifiManager.addParameter(&line3FilterParam);

  if (WiFi.psk().length() == 0) {
    // show screen wifi hint
    const int FONT_SIZE = global_settings.size_start_instruction_font;
    tft.setTextSize(0);
    tft.setTextColor(TFT_TEXT, TFT_BG);
    tft.setCursor(0, 0, FONT_SIZE);
    tft.print(cpp_instruction);
  }


  // Attempt to connect to Wi-Fi
  bool isConnected = wifiManager.autoConnect(cpp_ssid.c_str());

  new_config.SetLinesFilter(filterParam.getValue());
  new_config.SetLinesRBL(rblParam.getValue());
  new_config.SetLinesCount(countParam.getValue());
  new_config.SetLineName(1, line1Param.getValue());
  new_config.SetLineName(2, line2Param.getValue());
  new_config.SetLineName(3, line3Param.getValue());
  new_config.SetLine3RBL(line3RblParam.getValue());
  new_config.SetLine3Filter(line3FilterParam.getValue());

  Serial.println(new_config.GetLinesFilterAsString());
  Serial.println(new_config.GetLinesRblAsString());
  Serial.println(new_config.GetLinesCountAsString());
  Serial.println("Line 1: " + new_config.GetLineName(1));
  Serial.println("Line 2: " + new_config.GetLineName(2));
  Serial.println("Line 3: " + new_config.GetLineName(3));
  Serial.println("Line 3 RBL: " + new_config.GetLine3RBL());
  Serial.println("Line 3 Filter: " + new_config.GetLine3Filter());

  if (!isConnected) {
    Serial.println("Failed to connect");
    ESP.restart();
  } else {
    // Save the configuration to EEPROM
    tft.fillScreen(TFT_BG);
    if (old_config != new_config) {
      global_settings.SetConfig(new_config);
    }
    Serial.println("Saved!");
  }
}


/**
 * @brief Prints system information including free heap memory.
 */
void PrintSystemInfo() {
  // Get the free heap memory
  uint32_t freeHeap = ESP.getFreeHeap();
  Serial.print("FreeHeap:");
  Serial.print(freeHeap);
  Serial.print(", ");
  Serial.println();
}


/**
 * @brief Task to monitor and handle reset button presses.
 * @param pvParameters Pointer to task-specific parameters.
 */
void ResetActionsTask(void* pvParameters) {
  // Define the button pin
  const int buttonPin = global_settings.pin_reset_button;

  // Set the button pin as an input
  pinMode(buttonPin, INPUT);
  // Initialize button state to HIGH (assuming active LOW button)
  int buttonState = HIGH;
  // Counter for consecutive LOW readings
  int consecutiveLowReads = 0;
  const int& hard_threshold = global_settings.sec_press_hard_resset_button;
  const int& soft_threshold = global_settings.sec_press_soft_resset_button;

  while (1) {
    // Read the state of the button
    buttonState = digitalRead(buttonPin);

    // Check if the button is pressed (LOW)
    if (buttonState == LOW) {
      consecutiveLowReads++;
    } else {
      if (consecutiveLowReads > soft_threshold
          && consecutiveLowReads <= hard_threshold) {
        // Perform softReset here
        Serial.println("SOFT RESETTING");
        WiFiManager wifiManager;
        wifiManager.resetSettings();

        ESP.restart();
      }
      consecutiveLowReads = 0;  // Reset the consecutive LOW reads counter
    }

    if (consecutiveLowReads > hard_threshold) {
      // Reset WiFi settings and restart the ESP

      Serial.println("HARD RESETTING");
      global_settings.DeleteFile();
      WiFiManager wifiManager;
      wifiManager.resetSettings();

      ESP.restart();
    }
    unsigned long currentMillis = millis();
    if (currentMillis >= global_settings.ms_reboot_interval) {
      ESP.restart();
    }

    int delay_ms = global_settings.ms_task_delay_resset_button;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));  // Delay for 1 second
    PrintSystemInfo();
  }
}


/**
 * @brief Setup function for initializing the application.
 */
void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  Serial.println("APP START");

  // Initialize TFT display
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BG);

  // Initialize SPIFFS (SPI Flash File System)
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    delay(global_settings.ms_error_resset_delay);
    ESP.restart();  // Restart the ESP32
  }

  // Create a task for reading the reset button state
  xTaskCreate(ResetActionsTask, "ResetActionsTask", 2048 * 4, NULL, 1, NULL);

  // Run WiFiManagerTask to manage WiFi connection
  WiFiManagerTask();

  pTraficManager = new TraficManager;
  p_screen = new Screen(global_settings.GetConfig().GetLinesCountAsInt(),
                        global_settings.cnt_screen_lines_per_rows);

  // Check if WiFi is successfully connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi!");
    delay(global_settings.ms_error_resset_delay);
    ESP.restart();  // Restart the ESP32
  }

  // Create a mutex for synchronization
  dataMutex = xSemaphoreCreateMutex();

  // Create tasks for data updating and screen updating
  xTaskCreate(UpdateDataTask, "UpdateDataTask", 2048 * 64, NULL, 2, NULL);
  xTaskCreate(ScreenUpdateTask, "ScreenUpdateTask", 2048 * 16, NULL, 1, NULL);
}


/**
 * @brief Main loop function (not actively used in this application).
 */
void loop() {
  // This loop is intentionally left empty since the application is task-based.
}
