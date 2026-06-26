# AI 宸ョ▼浜ゆ帴涓婁笅鏂?
鏈枃妗ｇ敤浜庨厤鍚?`git log` 蹇€熸帴鎵嬪綋鍓嶅伐绋嬶紝鐩爣鏄鏂扮殑 AI 鎴栧紑鍙戣€呬笉闇€瑕侀噸鏂版懜绱?CubeMX銆並eil銆丗DCAN 鍜?M2006 鎺у埗閾捐矾銆?
## 椤圭洰姒傝

- 宸ョ▼璺緞锛歚D:\desktop\2026RC\Control\single_motor_test`
- 褰撳墠鍩虹嚎鎻愪氦锛歚f85d987 鍒濆鍖?M2006 鍗曠數鏈?CubeMX 鎺у埗宸ョ▼`
- 宸ョ▼鎬ц川锛氱湡瀹?STM32CubeMX 鐢熸垚宸ョ▼锛屼笉鏄墜宸ヤ豢閫犲伐绋嬬粨鏋勩€?- 鐩爣鑺墖锛歚STM32H723ZGTx`
- IDE/宸ュ叿閾撅細`MDK-ARM`锛孠eil 浣跨敤 ArmClang/AC6銆?- RTOS锛欳MSIS-RTOS2 / FreeRTOS銆?- 搴旂敤浠ｇ爜璇█锛欳銆?- 褰撳墠鍔熻兘锛氶€氳繃 CAN3 鎺у埗鍗曚釜 DJI M2006 鐢垫満鍜?C610 鐢佃皟锛屾敮鎸佺數娴併€侀€熷害銆佷綅缃笁绉嶈皟璇曟ā寮忋€?
## 鍏抽敭鏂囦欢

- `single_motor_test.ioc`锛欳ubeMX 宸ョ▼婧愭枃浠讹紝FDCAN3銆丗reeRTOS銆丳F6/PF7 绛夌‖浠堕厤缃粠杩欓噷鐢熸垚銆?- `MDK-ARM/single_motor_test.uvprojx`锛欿eil 宸ョ▼鏂囦欢锛屽凡璁剧疆 `<uAC6>1</uAC6>`锛屼笉瑕侀殢鎰忔敼鍥?ARMCC5銆?- `Core/Inc/dji_motor_task.h`锛歁2006 鎺у埗妯″紡銆並eil Watch 璋冭瘯缁撴瀯浣?`DJI_Motor_Debug`銆佸叏灞€鍙橀噺 `g_dji_motor_debug`銆?- `Core/Src/dji_motor_task.c`锛歊TOS 鐢垫満浠诲姟銆丆AN3 鏀跺彂銆侀€熷害鐜?浣嶇疆鐜皟搴︺€佽皟璇曞彉閲忔洿鏂般€?- `Core/Inc/dji_motor.h` 鍜?`Core/Src/dji_motor.c`锛欴JI 鐢垫満閫氱敤鍙嶉瑙ｆ瀽銆佺數娴佸崟浣嶆崲绠椼€佹帶鍒跺抚 ID/slot 鎵撳寘锛屽綋鍓嶆敮鎸?M2006/C610 鍜?M3508/C620銆?- `Core/Inc/pid.h` 鍜?`Core/Src/pid.c`锛氬閲忓紡 PID 鍜屼綅缃紡 PID锛屼娇鐢ㄧ湡瀹?`dt_s`锛屾病鏈夐殣钘?x10/x100 缂╂斁銆?- `tests/pc/test_m2006_control.c`锛歅C 渚у崟鍏冩祴璇曪紝瑕嗙洊 PID銆佸弽棣堣В鏋愩€佺數娴佹崲绠楀拰鎺у埗甯ф墦鍖呫€?- `.vscode/tasks.json`锛歏S Code 浠诲姟锛屽寘鍚?PC 娴嬭瘯鍜?Keil 鏋勫缓鍏ュ彛銆?
## M2006 鎺у埗閾捐矾

- CAN 閫氶亾锛欶DCAN3銆?- 寮曡剼锛歅F6 涓?FDCAN3_RX锛孭F7 涓?FDCAN3_TX銆?- 甯ф牸寮忥細Classic CAN锛?-byte data frame銆?- 鐢佃皟锛欳610锛孖D 涓?1銆?- 鎺у埗甯э細鏍囧噯甯?`0x200`锛孖D1 瀵瑰簲绗?1 涓數鏈?slot锛屽嵆 data[0..1]銆?- 鍙嶉甯э細鏍囧噯甯?`0x201`銆?- 鐢垫祦鍗曚綅锛氬澶栬皟璇曚娇鐢?`A`锛屽唴閮ㄥ彂閫佸墠杞崲涓?C610 raw current銆?- 瑙掑害鍗曚綅锛歚deg`銆?- 閫熷害鍗曚綅锛歚rpm`銆?- 鍑忛€熸瘮锛歚36.0f`銆?- 褰撳墠榛樿鎹㈢畻锛歚10A` 瀵瑰簲 C610 raw `16384`銆?
## Keil Watch 璋冭瘯鍏ュ彛

Keil Watch 鍙渶瑕佹坊鍔狅細

```c
g_dji_motor_debug
```

涓昏瀛楁锛?
- `enable`锛氭€讳娇鑳姐€俙false` 鏃跺彂閫?0A锛屽苟閲嶇疆 PID 鐘舵€併€?- `mode`锛氭帶鍒舵ā寮忥紝`0` 鍋滄锛宍1` 鐢垫祦妯″紡锛宍2` 閫熷害妯″紡锛宍3` 浣嶇疆妯″紡銆?- `target_current_A`锛氱數娴佹ā寮忕洰鏍囩數娴併€?- `target_speed_rpm`锛氶€熷害妯″紡鐩爣閫熷害锛涗綅缃ā寮忎笅鐢变綅缃幆杈撳嚭鏇存柊銆?- `target_angle_deg`锛氫綅缃ā寮忕洰鏍囨€昏搴︺€?- `feedback_speed_rpm`锛氬弽棣堣緭鍑鸿酱閫熷害銆?- `feedback_angle_deg`锛氬弽棣堝崟鍦堣緭鍑鸿酱瑙掑害銆?- `feedback_total_angle_deg`锛氬弽棣堢疮璁¤緭鍑鸿酱瑙掑害銆?- `feedback_current_A`锛氬弽棣堢數娴併€?- `feedback_temperature_c`锛氱數鏈烘俯搴︺€?- `command_current_A`锛氭渶缁堝彂閫佺數娴侊紝鍗曚綅 A銆?- `command_current_raw`锛氭渶缁堝彂閫佺數娴?raw 鍊笺€?- `rx_count`銆乣tx_count`銆乣error_count`锛欳AN 鏀跺彂鍜岄敊璇鏁般€?- `speed_pid_params`锛氶€熷害鐜?PID 鍙傛暟锛岃繍琛屾椂鍙湪 Watch 涓皟鑺傘€?- `angle_pid_params`锛氫綅缃幆 PID 鍙傛暟锛岃繍琛屾椂鍙湪 Watch 涓皟鑺傘€?
鎺ㄨ崘棣栨涓婄數璋冭瘯椤哄簭锛?
1. `mode = 1`锛屽皬鐢垫祦娴嬭瘯鏂瑰悜鍜屽弽棣堟槸鍚︽甯搞€?2. `mode = 2`锛屼粠 `target_speed_rpm = 100` 浠ヤ笅寮€濮嬮獙璇侀€熷害闂幆銆?3. `mode = 3`锛岀‘璁ょ疮璁¤搴︽柟鍚戞纭悗鍐嶆祴璇曚綅缃棴鐜€?4. 濡傛灉鍝嶅簲澶蒋锛屼紭鍏堝皬姝ユ彁楂?`speed_pid_params.kp` 鍜?`speed_pid_params.output_limit`銆?
## 鎺у埗棰戠巼鍜?PID

- 閫熷害鐜細1kHz锛屽閲忓紡 PID锛岃緭鍏?`target_speed_rpm` 鍜?`feedback_speed_rpm`锛岃緭鍑?`command_current_A`銆?- 浣嶇疆鐜細100Hz锛屼綅缃紡 PID锛岃緭鍏?`target_angle_deg` 鍜?`feedback_total_angle_deg`锛岃緭鍑?`target_speed_rpm`銆?- PID 浣跨敤鐪熷疄 `dt_s`锛氶€熷害鐜?`0.001f`锛屼綅缃幆 `0.01f`銆?- PID 鍙傛暟鍗曚綅鍜岃緭鍑哄崟浣嶇洿鎺ュ搴旇皟璇曞彉閲忥紝涓嶅仛闅愯棌 x10/x100 缂╂斁銆?- 褰撳墠榛樿鍙傛暟鍋忎繚瀹堬紝鐩爣鏄畨鍏ㄨ捣杞拰鏂逛究璋冭瘯锛屼笉杩芥眰寮€绠遍珮閫熷搷搴斻€?
## 鏋勫缓鍜屾祴璇?
PC 鍗曞厓娴嬭瘯锛?
```powershell
gcc -std=c99 -Wall -Wextra -ICore/Inc tests/pc/test_m2006_control.c Core/Src/pid.c Core/Src/dji_motor.c -o tests/pc/test_m2006_control.exe
.\tests\pc\test_m2006_control.exe
```

鏈熸湜缁撴灉锛?
```text
all tests passed
```

Keil 鏋勫缓锛?
```powershell
D:\Keil_v5\UV4\UV4.exe -b MDK-ARM\single_motor_test.uvprojx -j0
```

鏋勫缓鏃ュ織浣嶇疆锛?
```text
MDK-ARM\single_motor_test\single_motor_test.build_log.htm
```

鏈熸湜缁撴灉鍖呭惈锛?
```text
"single_motor_test\single_motor_test.axf" - 0 Error(s)
```

## Git 鍜屽崗浣滆鍒?
- 闄ら潪鐢ㄦ埛鏄庣‘瑕佹眰鎻愪氦 git锛屽惁鍒欎笉瑕佽嚜鍔ㄦ彁浜ゃ€?- 鐢ㄦ埛鏄庣‘瑕佹眰鎻愪氦鏃讹紝闇€瑕佸啓璇︾粏涓枃鎻愪氦淇℃伅銆?- 鎻愪氦鍚庡繀椤绘鏌?`git log -1 --format=fuller` 鍜?`git cat-file -p HEAD`锛岀‘璁や腑鏂囨棤涔辩爜銆佹帓鐗堟棤閿欎綅銆?- 鏋勫缓浜х墿銆並eil 鐢ㄦ埛鐣岄潰涓存椂鏂囦欢鍜?PC 娴嬭瘯 exe 鐢?`.gitignore` 蹇界暐銆?- `MDK-ARM/single_motor_test.uvoptx` 鏄?Keil 宸ョ▼閫夐」鏂囦欢锛屽彲鑳藉寘鍚?Watch 閰嶇疆锛涘鏋滃叾涓褰曚簡 `g_dji_motor_debug`锛屽彲浠ヤ綔涓鸿皟璇曚究鍒╅厤缃彁浜ゃ€?
## 褰撳墠娉ㄦ剰浜嬮」

- `MDK-ARM/single_motor_test.uvprojx` 涓殑 `<uAC6>1</uAC6>` 寰堥噸瑕併€傚綋鍓?FreeRTOS port 浣跨敤 clang/GCC 椋庢牸鍐呰仈姹囩紪锛屾敼鍥?ARMCC5 浼氬鑷?`portmacro.h` 缂栬瘧澶辫触銆?- `MDK-ARM/EventRecorderStub.scvd`銆乣MDK-ARM/single_motor_test/`銆乣MDK-ARM/startup_stm32h723xx.lst`銆乣tests/pc/test_m2006_control.exe` 鏄彲鍐嶇敓鎴愪骇鐗╋紝涓嶅簲鎻愪氦銆?- 濡傛灉閲嶆柊鐢?CubeMX 鐢熸垚浠ｇ爜锛岃閲嶇偣妫€鏌ョ敤鎴蜂唬鐮佸尯鏄惁淇濈暀锛屼互鍙?`pid.c`銆乣dji_motor.c`銆乣dji_motor_task.c` 鏄惁浠嶅湪 Keil 宸ョ▼鏂囦欢涓€?- 濡傛灉 CAN 娌℃湁鍙嶉锛屼紭鍏堟鏌ョ墿鐞?CAN 绾裤€佺粓绔數闃汇€丆610 ID銆丗DCAN3 寮曡剼 PF6/PF7 鍜屾护娉㈠櫒 `0x201`銆?
## 鏂?AI 鎺ユ墜娴佺▼

1. 鍏堣 `git log --oneline --decorate -5`锛岀‘璁ゅ綋鍓嶆彁浜ゅ巻鍙层€?2. 鍐嶈鏈枃妗ｏ紝寤虹珛宸ョ▼涓婁笅鏂囥€?3. 鏌ョ湅 `git status --short --ignored`锛屽尯鍒嗘簮鐮佹敼鍔ㄥ拰琚拷鐣ユ瀯寤轰骇鐗┿€?4. 闇€瑕佹敼鎺у埗閫昏緫鏃讹紝浼樺厛鏌ョ湅 `Core/Src/dji_motor_task.c`銆?5. 闇€瑕佹敼鍗忚瑙ｆ瀽鎴栫數娴佹崲绠楁椂锛屾煡鐪?`Core/Src/dji_motor.c`銆?6. 闇€瑕佹敼 PID 琛屼负鏃讹紝鏌ョ湅 `Core/Src/pid.c` 骞跺悓姝ユ洿鏂?PC 娴嬭瘯銆?7. 瀹屾垚淇敼鍚庤嚦灏戣繍琛?PC 鍗曞厓娴嬭瘯锛涙秹鍙?Keil 宸ョ▼鎴栧祵鍏ュ紡缂栬瘧鏃跺啀杩愯 Keil 鏋勫缓銆?8. 鍙湁鐢ㄦ埛鏄庣‘瑕佹眰鏃舵墠鎻愪氦锛屽苟鎸変腑鏂囪缁嗘彁浜や俊鎭鍒欐墽琛屻€?
## Keil source rename pitfall

- When renaming a source file, update both `MDK-ARM/single_motor_test.uvprojx` and `MDK-ARM/single_motor_test.uvoptx`; either file can keep the old `.c` path.
- Do not rely only on C includes or PC-side gcc tests. Re-run Keil build and inspect `MDK-ARM/single_motor_test/single_motor_test.build_log.htm`.
- The build log must show `compiling <new_name>.c...`; the typical failure is `armclang: error: no such file or directory: '../Core/Src/<old_name>.c'`.
- Concrete example: after renaming `dji_m2006.c` to `dji_motor.c`, both Keil project files must reference `../Core/Src/dji_motor.c`.

