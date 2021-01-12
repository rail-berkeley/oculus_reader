//
// Created by Jedrzej on 12/16/2020.
//

#include "Buttons.h"

namespace OVRFW {
    void Buttons::update_buttons(
            ovrInputStateTrackedRemote remoteInputState, const ovrHandedness controllerHand) {
        if (controllerHand == VRAPI_HAND_LEFT) {
            leftRemoteInputState_ = remoteInputState;
            leftAvailable_ = true;
        }
        else {
            rightRemoteInputState_ = remoteInputState;
            rightAvailable_ = true;
        }
    }

    std::string Buttons::current_to_string() {
        std::string text = "";
        // right hand
        if (rightAvailable_) {
            text += "R,"; // right hand available
            ovrInputStateTrackedRemote remoteInputState = rightRemoteInputState_;
            if (remoteInputState.Buttons & ovrButton_A)
                text += "A,";
            if (remoteInputState.Buttons & ovrButton_B)
                text += "B,";
            if (remoteInputState.Buttons & ovrButton_Trigger)
                text += "RTr,";
            if (remoteInputState.Buttons & ovrButton_GripTrigger)
                text += "RG,";
            if (remoteInputState.Touches & ovrTouch_ThumbUp)
                text += "RThU,";
            if (remoteInputState.Buttons & ovrButton_Joystick)
                text += "RJ,";
            text += "rightJS " + std::to_string(remoteInputState.Joystick.x) + " " +
                    std::to_string(remoteInputState.Joystick.y) + ",";
            text += "rightTrig " + std::to_string(remoteInputState.IndexTrigger) + ",";
            text += "rightGrip " + std::to_string(remoteInputState.GripTrigger);
        }
        // left hand
        if (leftAvailable_) {
            if (rightAvailable_)
                text += ",";
            text += "L,"; // left hand available
            ovrInputStateTrackedRemote remoteInputState = leftRemoteInputState_;
            if (remoteInputState.Buttons & ovrButton_X)
                text += "X,";
            if (remoteInputState.Buttons & ovrButton_Y)
                text += "Y,";
            if (remoteInputState.Buttons & ovrButton_Trigger)
                text += "LTr,";
            if (remoteInputState.Buttons & ovrButton_GripTrigger)
                text += "LG,";
            if (remoteInputState.Touches & ovrTouch_ThumbUp)
                text += "LThU,";
            if (remoteInputState.Buttons & ovrButton_Joystick)
                text += "LJ,";
            text += "leftJS " + std::to_string(remoteInputState.Joystick.x) + " " +
                    std::to_string(remoteInputState.Joystick.y) + ",";
            text += "leftTrig " + std::to_string(remoteInputState.IndexTrigger) + ",";
            text += "leftGrip " + std::to_string(remoteInputState.GripTrigger);
        }
        return text;
    }
}