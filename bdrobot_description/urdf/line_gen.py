from PIL import Image, ImageDraw

# 设置图像尺寸（正方形，便于对称显示十字）
image_size = 80  # 图像边长，单位为像素
background_color = (0, 0, 0, 0)  # 透明背景（RGBA格式，最后一位为透明度）
line_color = (0, 255, 0)  # 绿色（RGB格式，0-255范围）
line_width = 2  # 十字线宽度

# 创建透明背景的图像
image = Image.new('RGBA', (image_size, image_size), background_color)
draw = ImageDraw.Draw(image)

# 计算十字中心坐标
center_x = image_size // 2

# 绘制垂直线（十字的竖）
# 从图像上边缘到下边缘，居中显示
draw.line(
    [(center_x, 0), (center_x, image_size)],  # 起点和终点坐标
    fill=line_color,
    width=line_width
)

# 保存为PNG文件
output_path = "green_line.png"
image.save(output_path)

print(f"十字激光图像已保存至：{output_path}")
 
