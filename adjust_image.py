from PIL import Image, ImageDraw, ImageFont

image_path = "/home/jesse/Downloads/image00001.jpeg"
font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"  # Default system font path
font = ImageFont.truetype(font_path, 30)


# Adjust box coordinates based on the image size and current alignment issues
final_boxes = {
    "Operator Area": [(5, 650), (1620, 1430)],  # Adjusted lower and larger to fit operator area better
    "KUKA Arm": [(1180, 440), (1520, 800)],  # Shifted right and expanded to better fit the robotic arm
    "Secondary Task": [(1760, 550), (2280, 800)]  # Moved right and slightly expanded to cover the task area
}

# Adjust text positions to align well with boxes
final_annotations = {
    "Operator Area": (25, 665),
    "KUKA Arm": (1190, 450),
    "Secondary Task": (1775, 560)
}

# Adjust camera arrow position and label
final_arrow_start = (1224, 100)  # Adjusted arrow start point
final_arrow_end = (1224, 50)  # Adjusted arrow end point
final_camera_text_pos = (1160, 15)  # Adjusted text position for "Camera"

# Reload the original image to ensure clean annotations
image = Image.open(image_path)
draw = ImageDraw.Draw(image)

# Draw final adjusted boxes
for box in final_boxes.values():
    draw.rectangle(box, outline="red", width=5)

# Add final adjusted annotations for areas
for text, position in final_annotations.items():
    draw.text(position, text, fill="red", font=font)

# Draw final arrow for "Camera"
draw.line([final_arrow_start, final_arrow_end], fill="red", width=3)  # Draw the arrow line
draw.polygon([(final_arrow_end[0] - 10, final_arrow_end[1] + 10),
              (final_arrow_end[0] + 10, final_arrow_end[1] + 10),
              final_arrow_end], fill="red")  # Draw the arrowhead
draw.text(final_camera_text_pos, "Camera", fill="red", font=font)

# Save the final adjusted image
final_adjusted_image_path = "/home/jesse/annotated_setup.jpeg"
image.save(final_adjusted_image_path)
final_adjusted_image_path
