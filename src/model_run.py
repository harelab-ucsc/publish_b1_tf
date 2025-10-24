
import onnxruntime as ort
import numpy as np
import time


inference_session = ort.InferenceSession("./2025-05-01_02-08-14/exported/policy.onnx")

input = np.random.rand(1, 48).astype("float32")
output = inference_session.run(None, {"obs": input})[0].tolist()[0]

# print("test output", output)

# performance check
iterations = 100
times = []

for _ in range(iterations):
    start_time = time.perf_counter()
        
    # input = np.random.rand(1, 48).astype("float32")
    output = inference_session.run(None, {"obs": input})[0].tolist()[0]

    end_time = time.perf_counter()
    
    elapsed_time = end_time - start_time
    times.append(elapsed_time)

# for i, elapsed in enumerate(times, 1):
#     print(f"Iteration {i}: {elapsed:.6f} seconds")

print(f"Frequency (Hz): {len(times) / sum(times):.2f}")


input = np.array([[
    0, 0, 0, # base lin vel (x, y, z)
    0, 0, 0, # base angular vel (pitch, roll, yaw)
    0, 0, -9.8, # projected gravity (x, y, z)
    0, 0, 0,# velocity command (x, y, omega)
    # Joint Pos (12)
    #FL        #FR        #RL        #RR
    0, 0, 0,   0, 0, 0,   0, 0, 0,   0, 0, 0,
    # joint vel (12)
    #FL        #FR        #RL        #RR
    0, 0, 0,   0, 0, 0,   0, 0, 0,   0, 0, 0,
    # actions (12)
    #FL        #FR        #RL        #RR
    0, 0, 0,   0, 0, 0,   0, 0, 0,   0, 0, 0
]]).astype("float32")

#FL, FR, RL, RR; Hip, Thigh, Calf

output = inference_session.run(None, {"obs": input})[0].tolist()[0]
print(len(output))
[print(f"{num:.2f}") for num in output]