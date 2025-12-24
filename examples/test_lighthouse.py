import pysurvive
import sys
import time

print("初始化 pysurvive...")
ctx = pysurvive.SimpleContext(sys.argv)

print("等待设备检测...")
print("(请确保 Tracker 在基站的可视范围内)")
print()

# 收集检测到的设备
detected_devices = {}
start_time = time.time()
timeout = 20  # 20秒超时

while time.time() - start_time < timeout:
    # 检查是否有位姿更新
    updated = ctx.NextUpdated()
    if updated:
        name = str(updated.Name(), "utf-8")
        if name not in detected_devices:
            detected_devices[name] = {"first_seen": time.time() - start_time}
            print(f"[{detected_devices[name]['first_seen']:.1f}s] 新设备: {name}")
        
        # 获取位姿
        try:
            poseObj = updated.Pose()
            if poseObj:
                poseData = poseObj[0]
                detected_devices[name]["last_pose"] = {
                    "pos": (poseData.Pos[0], poseData.Pos[1], poseData.Pos[2]),
                    "rot": (poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3])
                }
        except Exception as e:
            pass
    
    # 每3秒打印状态
    elapsed = time.time() - start_time
    if int(elapsed) % 3 == 0 and int(elapsed) > 0:
        time.sleep(0.1)  # 避免重复打印
        
        lighthouses = [n for n in detected_devices if n.startswith("LH")]
        trackers = [n for n in detected_devices if n.startswith("WM") or n.startswith("T2")]
        
        if len(lighthouses) >= 2 and len(trackers) >= 2:
            break

# 最终结果
print("\n" + "="*50)
print("检测结果:")
print("="*50)

lighthouses = []
trackers = []
others = []

for name, info in detected_devices.items():
    if name.startswith("LH"):
        lighthouses.append(name)
    elif name.startswith("WM") or name.startswith("T2") or name.startswith("HMD"):
        trackers.append(name)
        if "last_pose" in info:
            pos = info["last_pose"]["pos"]
            print(f"  {name} 位置: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
    else:
        others.append(name)

print(f"\n基站 (Lighthouses): {lighthouses if lighthouses else '无'}")
print(f"Trackers: {trackers if trackers else '无'}")
if others:
    print(f"其他设备: {others}")

print()
if len(lighthouses) >= 2 and len(trackers) >= 2:
    print("✅ 检测到两个基站和两个 Tracker！系统就绪。")
elif len(lighthouses) >= 2 and len(trackers) == 1:
    print("⚠️ 只检测到一个 Tracker。另一个可能未配对或不在可视范围内。")
elif len(lighthouses) >= 2 and len(trackers) == 0:
    print("❌ 未检测到 Tracker。请检查:")
    print("   1. Tracker 是否已与 Dongle 配对 (LED 应为绿色)")
    print("   2. Tracker 是否在基站的可视范围内")
    print("   3. 尝试移动 Tracker")
elif len(lighthouses) < 2:
    print(f"❌ 只检测到 {len(lighthouses)} 个基站")
