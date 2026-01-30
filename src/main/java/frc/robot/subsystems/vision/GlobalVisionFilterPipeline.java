package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;

public class GlobalVisionFilterPipeline {

  // record class for holding individual filters
  private record Filter(String name, GlobalVisionFilter filter) {}

  private final String m_logPrefix = "vision/globalFilters/";

  private List<Filter> allFilters;

  // HashMap of all filter enabled states (filterName, isEnabled)
  private Map<String, Boolean> enabledFilters = new HashMap<>();

  public GlobalVisionFilterPipeline() {
      allFilters = new ArrayList<>();
  }

  public List<String> getFilterNames() {
      return allFilters.stream().map(Filter::name).toList();
  }

  public int getNumFilters() {
    return allFilters.size();
  }

  // Adds a new filter to the pipeline
  public void addFilter(String name, GlobalVisionFilter filter) {
      allFilters.add(new Filter(name, filter));
      enabledFilters.put(name, true);
  }

  /**
   * Runs all enabled filters on a reading.
   * Logs each filter's result to SmartDashboard.
   *
   * @param allReadings All camera readings to compare against
   * @return true if reading passes ALL enabled filters, false otherwise
   */
  public List<CameraReading> runAll(List<CameraReading> allReadings) {
    List<CameraReading> validReadings = allReadings;

    // loop through all filterMethods + runs on reading
    for (Filter filterMethod : allFilters) {
      String filterName = filterMethod.name();
      GlobalVisionFilter filter = filterMethod.filter();

      // If filter not enabled skip
      if (!isEnabled(filterName)) {
        // posts "skipped" to SmartDashboard (NOTE: this is DIFFERENT from the filter "enabled" toggles)
        SmartDashboard.putString(m_logPrefix + "allCameras/" + filterName + "/result", "skipped");
        continue;
      }

      validReadings = filter.validReadings(validReadings);

      // log result to SmartDashboard
      SmartDashboard.putString(m_logPrefix + "allCameras/" + filterName + "/result", (validReadings.size() > 0 ? "valid readings" : "no valid readings"));

      // short circuit if ANY filter fails
      if (validReadings.size() == 0) {
          return validReadings;
      }
    }
    return validReadings;
  }


  // ----- SMARTDASHBOARD -----

  /*
    * Checks if a filter is enabled by SmartDashboard toggle
    * @return true if enabled, false otherwise (defaults to false if filter not found)
    */
  public boolean isEnabled(String filterName) {
    return enabledFilters.getOrDefault(filterName, false);
  }

  /**
   * Updates the enabled state of all filters based on SmartDashboard toggles.
   */
  public void updateSmartDashboardToggles() {
    for (Filter filter : allFilters) {
        String filterName = filter.name();

        // read toggle enabled data from SmartDashboard
        // if it doesn't find the filter, it defaults to true (enabled)
        // (^^ important so that new filters default to on)
        boolean currentState = SmartDashboard.getBoolean(m_logPrefix + filterName + "/enabled", true);
        enabledFilters.put(filterName, currentState);
    }
  }
}
