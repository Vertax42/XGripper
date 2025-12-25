import sys
import time
from xensesdk import ExampleView
from xensesdk import Sensor
from xensesdk import call_service

def main():
    MASTER_SERVICE = "master_6ebbc5f53240"
    mac_addr = MASTER_SERVICE.split("_")[-1]
    
    # find all sensors
    ret = call_service(MASTER_SERVICE, "scan_sensor_sn")
    if ret is None:
        print(f"Failed to scan sensors")
        sys.exit(1)
    
    sensor_sns = list(ret.keys())
    print(f"Found {len(sensor_sns)} sensors: {sensor_sns}")
    
    if len(sensor_sns) < 2:
        print("Error: Need at least 2 sensors")
        sys.exit(1)

    # create both sensors
    sensor_0 = Sensor.create(sensor_sns[0], mac_addr=mac_addr, rectify_size=(400, 700))
    sensor_1 = Sensor.create(sensor_sns[1], mac_addr=mac_addr, rectify_size=(400, 700))
    print(f"Created sensor 0: {sensor_sns[0]}")
    print(f"Created sensor 1: {sensor_sns[1]}")
    
    # Create view with first sensor (for callback mechanism)
    View = ExampleView(sensor_0)
    View2d_0 = View.create2d(Sensor.OutputType.Difference, Sensor.OutputType.Rectify)
    View2d_1 = View.create2d(Sensor.OutputType.Difference, Sensor.OutputType.Rectify)
    
    # FPS tracking
    frame_count = 0
    last_fps_time = time.time()
    fps_update_interval = 10  # Update FPS every N frames
    
    # Timing accumulators
    time_sensor0_read = 0.0
    time_sensor0_view = 0.0
    time_sensor1_read = 0.0
    time_sensor1_view = 0.0
    @profile
    def callback():
        nonlocal frame_count, last_fps_time
        nonlocal time_sensor0_read, time_sensor0_view, time_sensor1_read, time_sensor1_view
        
        # Read from sensor 0
        t0 = time.perf_counter()
        diff_0, rectify_0 = sensor_0.selectSensorInfo(Sensor.OutputType.Difference, Sensor.OutputType.Rectify)
        t1 = time.perf_counter()
        View2d_0.setData(Sensor.OutputType.Difference, diff_0)
        View2d_0.setData(Sensor.OutputType.Rectify, rectify_0)
        t2 = time.perf_counter()
        
        # Read from sensor 1
        diff_1, rectify_1 = sensor_1.selectSensorInfo(Sensor.OutputType.Difference, Sensor.OutputType.Rectify)
        t3 = time.perf_counter()
        View2d_1.setData(Sensor.OutputType.Difference, diff_1)
        View2d_1.setData(Sensor.OutputType.Rectify, rectify_1)
        t4 = time.perf_counter()
        
        # Accumulate timing
        time_sensor0_read += (t1 - t0)
        time_sensor0_view += (t2 - t1)
        time_sensor1_read += (t3 - t2)
        time_sensor1_view += (t4 - t3)
        
        # Calculate and print FPS
        frame_count += 1
        if frame_count % fps_update_interval == 0:
            current_time = time.time()
            fps = fps_update_interval / (current_time - last_fps_time)
            last_fps_time = current_time
            
            # Calculate average timing (in ms)
            avg_s0_read = (time_sensor0_read / fps_update_interval) * 1000
            avg_s0_view = (time_sensor0_view / fps_update_interval) * 1000
            avg_s1_read = (time_sensor1_read / fps_update_interval) * 1000
            avg_s1_view = (time_sensor1_view / fps_update_interval) * 1000
            total = avg_s0_read + avg_s0_view + avg_s1_read + avg_s1_view
            
            print(f"\rFPS: {fps:.1f} | S0 read: {avg_s0_read:.1f}ms, view: {avg_s0_view:.1f}ms | "
                  f"S1 read: {avg_s1_read:.1f}ms, view: {avg_s1_view:.1f}ms | Total: {total:.1f}ms", 
                  end="", flush=True)
            
            # Reset accumulators
            time_sensor0_read = 0.0
            time_sensor0_view = 0.0
            time_sensor1_read = 0.0
            time_sensor1_view = 0.0
            
    View.setCallback(callback)

    View.show()
    print()  # New line after FPS output
    sensor_0.release()
    sensor_1.release()
    sys.exit()


if __name__ == '__main__':
    main()