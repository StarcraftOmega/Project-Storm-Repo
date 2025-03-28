from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    launch_description = LaunchDescription()
    
    # Helper function to create nodes with specific domain IDs
    def create_node_with_domain(package_name, executable_name, node_name, domain_id):
        return Node(
            package=package_name,
            executable=executable_name,
            name=node_name,
            output='screen',
            # Set environment variables for this node
            environment=[
                {'name': 'ROS_DOMAIN_ID', 'value': str(domain_id)}
            ]
        )
    
   # Node Configuration
    #node1 = create_node_with_domain(
    #    package_name='your_package_1',    # Replace with your package name
    #    executable_name='your_node_1',    # Replace with your executable name
    #    node_name='custom_node_name_1',   # Replace with desired node name
    #    domain_id=0                       # Set domain ID for this node
    #)
    
    node1 = create_node_with_domain(
        package_name='your_package_2',    
        executable_name='your_node_2',    
        node_name='custom_node_name_2',   
        domain_id=1                       
    )
    
    node2 = create_node_with_domain(
        package_name='your_package_3',    
        executable_name='your_node_3',    
        node_name='custom_node_name_3',  
        domain_id=2                       
    )
    
    # Add nodes to launch description
    launch_description.add_action(node1)
    launch_description.add_action(node2)
    
    # Optional: Set default domain ID for any nodes not explicitly set
    launch_description.add_action(
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0')
    )
    
    return launch_description
