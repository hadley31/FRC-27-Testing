# Third-Party Notices

This repository contains source code from several third-party open-source projects,
in addition to code written by this team. This file records what is included, where
it lives, and under which license.

All licenses below are permissive (BSD 3-Clause or MIT). None of them are copyleft,
and none require this team's own robot code to be published. Each one does require
that its copyright notice and license text be retained in redistributions of source
code, which is what this file and the [`licenses/`](licenses/) directory provide.

## Included projects

| Project | Where it lives in this repo | How it is used | License |
| --- | --- | --- | --- |
| **WPILib** | `src/main/java/first/Main.java`, `src/main/java/first/robot/Robot.java` (project template), plus the WPILib libraries resolved by Gradle | Library dependency + template files | BSD 3-Clause — [`WPILib-License.md`](WPILib-License.md) |
| **ChoreoLib** (Choreo) | `src/main/java/choreo/` | Vendored source | BSD 3-Clause — [`licenses/Choreo-LICENSE.txt`](licenses/Choreo-LICENSE.txt) |
| **BLine** | `src/main/java/BLine/` | Vendored source, modified | BSD 3-Clause — [`licenses/BLine-LICENSE.txt`](licenses/BLine-LICENSE.txt) |
| **PathPlanner** | `src/main/java/BLine/FlippingUtil.java` | Vendored source (reached via BLine, which copied this class) | MIT — [`licenses/PathPlanner-LICENSE.txt`](licenses/PathPlanner-LICENSE.txt) |
| **AdvantageKit** (Littleton Robotics) | `vendordeps/AdvantageKit.json`; libraries resolved by Gradle | Library dependency | BSD 3-Clause — [`licenses/AdvantageKit-LICENSE.txt`](licenses/AdvantageKit-LICENSE.txt) |
| **Mechanical Advantage robot code** (FRC 6328) | `src/main/java/first/lib/util/LoggedTracer.java`, `src/main/java/first/robot/util/PoseEstimator.java` | Vendored source | MIT — [`licenses/FRC6328-LICENSE.txt`](licenses/FRC6328-LICENSE.txt) |

Note that AdvantageKit (the library, BSD 3-Clause) and Mechanical Advantage's robot
code (MIT) are both published by Littleton Robotics / FRC 6328 but are licensed
separately, so both licenses are included.

## Notes on modifications

- `src/main/java/BLine/` has been modified from upstream BLine to target WPILib 2027
  and Commands v3: the command framework was migrated from Commands v2 to Commands v3,
  WPILib APIs were updated to their 2027 equivalents, and `JsonUtils` was reimplemented
  on Gson in place of json-simple.
- The two Mechanical Advantage files retain their original file headers. Those headers
  refer to "the LICENSE file at the root directory of this project", which for those
  files means the upstream project's license, reproduced here as
  [`licenses/FRC6328-LICENSE.txt`](licenses/FRC6328-LICENSE.txt).

## Other dependencies

Libraries that are resolved by Gradle and are not redistributed in this repository as
source -- including Phoenix 6 (CTRE), PhotonVision, Gson, and the Gradle wrapper -- keep
their own licenses in their respective distributions and are not reproduced here.
