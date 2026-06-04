from cad_to_pointcloud import CADToPointCloudConverter

# 创建转换器实例
converter = CADToPointCloudConverter(scale=0.1, point_density=0.5)

# 执行转换
pointcloud = converter.convert(
    input_file="/home/yuyouling/ezdxf/examples_dxf/hatches_1.dxf",
    output_file="output.pcd",
    height=3,
    visualize=True
)