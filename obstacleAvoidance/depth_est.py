import torch
import cv2
import numpy as np
from torchvision.transforms import Compose, ToTensor, Normalize
from PIL import Image

# Load MiDaS model
model_type = "DPT_BEiT_Large_512"  # Change to "MiDaS_small" for ONNX
# model_path = "/home/vinoth/ComputerVisionCPP/Workspace/obstacleAvoidance/dpt_beit_large_512.onnx"  # Change to "midas_small.onnx" if using ONNX
model_path = "/home/vinoth/ComputerVisionCPP/Workspace/obstacleAvoidance/midas_v21_small_256.onnx"
def load_model(model_type, model_path):
    if "onnx" in model_path:
        import onnxruntime
        return onnxruntime.InferenceSession(model_path), "onnx"
    else:
        model = torch.hub.load("intel-isl/MiDaS", model_type)
        model.load_state_dict(torch.load(model_path, map_location=torch.device("cpu")))
        model.eval()
        return model, "torch"

model, backend = load_model(model_type, model_path)

# Define preprocessing function
transform = Compose([
    lambda x: x.convert("RGB"),
    ToTensor(),
    Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

def estimate_depth(frame):
    # img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert to RGB
    img = cv2.resize(img, (256,256))  # Resize to model's expected input
    img = Image.fromarray(img)
    img = transform(img).unsqueeze(0)
    
    if backend == "torch":
        with torch.no_grad():
            depth = model(img).squeeze().numpy()
    else:  # ONNX runtime
        ort_inputs = {model.get_inputs()[0].name: img.numpy()}
        depth = model.run(None, ort_inputs)[0][0]
    
    depth = cv2.resize(depth, (frame.shape[1], frame.shape[0]))
    return depth

# def detect_obstacle(depth_map, threshold=0.3):
#     height, width = depth_map.shape
#     left = np.mean(depth_map[:, :width//3])
#     center = np.mean(depth_map[:, width//3:2*width//3])
#     right = np.mean(depth_map[:, 2*width//3:])
    
#     obstacle_direction = "None"
#     if center < threshold:
#         if left < right:
#             obstacle_direction = "Right"
#         else:
#             obstacle_direction = "Left"
#     elif left < threshold:
#         obstacle_direction = "Right"
#     elif right < threshold:
#         obstacle_direction = "Left"
    
#     return obstacle_direction
def detect_obstacle(depth_map, threshold=0.5):
    depth_map = depth_map / np.max(depth_map)  # Normalize
    depth_map = 1.0 / (depth_map + 1e-6)  # Invert Depth for correctness

    height, width = depth_map.shape

    # Split image into left, center, and right
    left_region = depth_map[:, :width//3]
    center_region = depth_map[:, width//3:2*width//3]
    right_region = depth_map[:, 2*width//3:]

    # Get the MINIMUM depth value in each region (closest object)
    left = np.min(left_region)
    center = np.min(center_region)
    right = np.min(right_region)

    print(f"Depth - Left: {left:.2f}, Center: {center:.2f}, Right: {right:.2f}")  # Debugging

    obstacle_direction = "None"
    
    # Detect where the closest obstacle is
    if center < threshold:
        obstacle_direction = "Center"
    if left < threshold and left < center and left < right:
        obstacle_direction = "Left"
    if right < threshold and right < center and right < left:
        obstacle_direction = "Right"
    
    return obstacle_direction


def main():
    cap = cv2.VideoCapture(0)  # Adjust camera index if necessary
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        depth_map = estimate_depth(frame)
        # direction = detect_obstacle(depth_map)
        direction = detect_obstacle(depth_map, threshold=1.0)  # Increase threshold

        cv2.putText(frame, f"Obstacle: {direction}", (50, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.imshow("Depth Map", cv2.applyColorMap(cv2.convertScaleAbs(depth_map, alpha=255.0/np.max(depth_map)), cv2.COLORMAP_JET))
        cv2.imshow("Frame", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
