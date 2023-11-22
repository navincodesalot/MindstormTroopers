package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class GamepadTrigger extends Trigger {

    private final GamepadEx m_gamepad;
    private final GamepadKeys.Trigger[] m_triggers;


    public GamepadTrigger(GamepadEx gamepad, @NonNull GamepadKeys.Trigger... triggers) {
        m_gamepad = gamepad;
        m_triggers = triggers;
    }

    @Override
    public boolean get() {
        boolean res = true;
        for (GamepadKeys.Trigger trigger : m_triggers)
            res = res && m_gamepad.getTrigger(trigger) > 0.5;
        return res;
    }

}