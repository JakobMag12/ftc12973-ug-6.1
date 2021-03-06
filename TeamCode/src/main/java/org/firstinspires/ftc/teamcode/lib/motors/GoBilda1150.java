

package org.firstinspires.ftc.teamcode.lib.motors;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev = 145.6, gearing = 5.2, maxRPM = 1150, orientation = Rotation.CCW, achieveableMaxRPMFraction = 1.0)
@DeviceProperties(xmlTag = "goBilda1150", name = "GoBILDA 1150", builtIn = false)
@DistributorInfo(distributor="goBILDA_distributor", model="goBILDA-5202", url="https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motors/")
@ExpansionHubPIDFVelocityParams(P=2.0, I=0.5, F=11.1)
@ExpansionHubPIDFPositionParams(P=5.0)
public interface GoBilda1150 {
}
