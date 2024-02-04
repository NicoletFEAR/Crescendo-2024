package frc.lib.utilities;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class ShuffleboardButton {
    
    private boolean defaultValue;
    private GenericEntry entry;

    public ShuffleboardButton(String key,
      boolean defaultValue,
      ShuffleboardTab tab,
      WidgetType widget,
      Map<String, Object> properties,
      int columnIndex,
      int rowIndex) {
        this.defaultValue = defaultValue;
        this.entry =
            tab.add(key, defaultValue)
                .withWidget(widget)
                .withProperties(properties)
                .withPosition(columnIndex, rowIndex)
                .getEntry();
    }

    public boolean getValue() {
        if (entry.getBoolean(defaultValue)) {
            entry.setBoolean(false);
            return true;
        }
        return false;
    }
}
