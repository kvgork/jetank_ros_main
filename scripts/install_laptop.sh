#!/bin/bash
################################################################################
# JeTank Remote Simulation - Laptop Installation Script
#
# This script installs all necessary dependencies for running Gazebo Fortress
# on your laptop for remote simulation with Jetson.
#
# Usage:
#   chmod +x install_laptop.sh
#   ./install_laptop.sh
#
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
DEFAULT_DOMAIN_ID=42
DOMAIN_ID=${DOMAIN_ID:-$DEFAULT_DOMAIN_ID}

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "${BLUE}"
    echo "================================================================================"
    echo "$1"
    echo "================================================================================"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

prompt_yes_no() {
    while true; do
        read -p "$1 (y/n): " yn
        case $yn in
            [Yy]* ) return 0;;
            [Nn]* ) return 1;;
            * ) echo "Please answer yes or no.";;
        esac
    done
}

################################################################################
# System Check
################################################################################

check_system() {
    print_header "Checking System Requirements"

    # Check OS
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        echo "OS: $NAME $VERSION"

        if [[ "$VERSION_ID" != "22.04" ]]; then
            print_warning "This script is designed for Ubuntu 22.04"
            print_warning "You are running: $NAME $VERSION"
            if ! prompt_yes_no "Continue anyway?"; then
                exit 1
            fi
        fi
    else
        print_error "Cannot detect OS version"
        exit 1
    fi

    # Check if ROS 2 Humble is installed
    if [ -f /opt/ros/humble/setup.bash ]; then
        print_success "ROS 2 Humble found"
    else
        print_error "ROS 2 Humble not found!"
        print_info "Please install ROS 2 Humble first: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi

    # Check internet connection
    if ping -c 1 packages.ros.org &> /dev/null; then
        print_success "Internet connection OK"
    else
        print_error "No internet connection. Please check your network."
        exit 1
    fi

    print_success "System check passed"
    echo ""
}

################################################################################
# Install Dependencies
################################################################################

install_dependencies() {
    print_header "Installing Gazebo Fortress Dependencies"

    print_info "Updating package lists..."
    sudo apt update

    print_info "Installing Gazebo Fortress packages..."

    PACKAGES=(
        "ros-humble-ros-gz-sim"
        "ros-humble-ros-gz-bridge"
        "ros-humble-ros-gz-image"
        "ros-humble-diff-drive-controller"
        "ros-humble-joint-state-broadcaster"
        "ros-humble-controller-manager"
        "ros-humble-xacro"
        "ros-humble-robot-state-publisher"
        "ros-humble-teleop-twist-keyboard"
        "ros-humble-rqt-image-view"
    )

    for package in "${PACKAGES[@]}"; do
        if dpkg -l | grep -q "^ii  $package"; then
            print_success "$package already installed"
        else
            print_info "Installing $package..."
            sudo apt install -y $package
            print_success "$package installed"
        fi
    done

    print_success "All dependencies installed"
    echo ""
}

################################################################################
# Network Configuration
################################################################################

configure_network() {
    print_header "Configuring Network for Remote Simulation"

    # Configure ROS_DOMAIN_ID
    print_info "Configuring ROS_DOMAIN_ID..."

    BASHRC="$HOME/.bashrc"
    DOMAIN_EXPORT="export ROS_DOMAIN_ID=$DOMAIN_ID"

    if grep -q "export ROS_DOMAIN_ID=" "$BASHRC"; then
        print_warning "ROS_DOMAIN_ID already set in .bashrc"
        CURRENT_ID=$(grep "export ROS_DOMAIN_ID=" "$BASHRC" | tail -1 | cut -d'=' -f2)
        print_info "Current setting: ROS_DOMAIN_ID=$CURRENT_ID"

        if prompt_yes_no "Update to ROS_DOMAIN_ID=$DOMAIN_ID?"; then
            # Comment out old lines
            sed -i 's/^export ROS_DOMAIN_ID=/#&/' "$BASHRC"
            # Add new line
            echo "$DOMAIN_EXPORT  # Added by jetank install script" >> "$BASHRC"
            print_success "Updated ROS_DOMAIN_ID=$DOMAIN_ID in .bashrc"
        fi
    else
        echo "$DOMAIN_EXPORT  # Added by jetank install script" >> "$BASHRC"
        print_success "Added ROS_DOMAIN_ID=$DOMAIN_ID to .bashrc"
    fi

    # Set for current session
    export ROS_DOMAIN_ID=$DOMAIN_ID

    # Configure RMW implementation
    print_info "Configuring ROS 2 middleware..."

    RMW_EXPORT="export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"

    if grep -q "export RMW_IMPLEMENTATION=" "$BASHRC"; then
        print_success "RMW_IMPLEMENTATION already configured"
    else
        echo "$RMW_EXPORT  # Added by jetank install script" >> "$BASHRC"
        print_success "Configured to use FastRTPS"
    fi

    # Install FastRTPS if needed
    if ! dpkg -l | grep -q "ros-humble-rmw-fastrtps-cpp"; then
        print_info "Installing FastRTPS..."
        sudo apt install -y ros-humble-rmw-fastrtps-cpp
        print_success "FastRTPS installed"
    fi

    print_success "Network configuration complete"
    echo ""
}

################################################################################
# Firewall Configuration
################################################################################

configure_firewall() {
    print_header "Configuring Firewall"

    # Check if ufw is installed
    if ! command -v ufw &> /dev/null; then
        print_info "UFW firewall not installed, skipping..."
        return
    fi

    # Check if ufw is active
    if sudo ufw status | grep -q "Status: active"; then
        print_warning "Firewall is currently active"
        echo ""
        echo "For remote simulation, you have two options:"
        echo ""
        echo "1. Disable firewall (easiest, less secure)"
        echo "   sudo ufw disable"
        echo ""
        echo "2. Open specific ports (more secure)"
        echo "   - ROS 2 DDS Discovery: UDP 7400-7500"
        echo "   - ROS 2 Data: UDP 30000-65535"
        echo ""

        if prompt_yes_no "Would you like to open ROS 2 ports now?"; then
            print_info "Opening ROS 2 DDS ports..."

            # Discovery ports
            sudo ufw allow 7400:7500/udp comment 'ROS 2 DDS Discovery'
            sudo ufw allow 7400:7500/tcp comment 'ROS 2 DDS Discovery'

            # Data ports
            sudo ufw allow 30000:65535/udp comment 'ROS 2 DDS Data'

            print_success "Firewall ports opened"

            print_warning "Note: You may need to do the same on your Jetson"
        else
            print_warning "Firewall not configured. You may need to:"
            print_info "  sudo ufw disable    (temporary)"
            print_info "OR configure ports manually (see REMOTE_SIMULATION_GUIDE.md)"
        fi
    else
        print_success "Firewall is disabled (good for testing)"
    fi

    echo ""
}

################################################################################
# Workspace Setup
################################################################################

setup_workspace() {
    print_header "Setting Up Workspace"

    # Check if we're in a ROS 2 workspace
    if [ -f "install/setup.bash" ]; then
        print_success "ROS 2 workspace detected"

        print_info "Sourcing workspace..."
        source install/setup.bash

        # Check for jetank packages
        if ros2 pkg list | grep -q "jetank_simulation"; then
            print_success "JeTank simulation package found"
        else
            print_warning "JeTank simulation package not found"
            print_info "Please build your workspace first:"
            print_info "  colcon build --packages-select jetank_simulation jetank_description jetank_ros2_main"
        fi
    else
        print_warning "Not in a ROS 2 workspace root"
        print_info "Please cd to your workspace and run this script again"
    fi

    echo ""
}

################################################################################
# Network Test
################################################################################

test_network() {
    print_header "Network Connectivity Test"

    echo "For remote simulation, both laptop and Jetson must be on the same network."
    echo ""

    if prompt_yes_no "Would you like to test connectivity to Jetson now?"; then
        read -p "Enter Jetson IP address (e.g., 192.168.1.101): " JETSON_IP

        if [ -n "$JETSON_IP" ]; then
            print_info "Testing connection to $JETSON_IP..."

            if ping -c 3 -W 2 "$JETSON_IP" &> /dev/null; then
                print_success "Successfully connected to Jetson at $JETSON_IP"

                # Measure latency
                LATENCY=$(ping -c 10 "$JETSON_IP" | tail -1 | awk -F '/' '{print $5}')
                print_info "Average latency: ${LATENCY}ms"

                if (( $(echo "$LATENCY < 50" | bc -l) )); then
                    print_success "Latency is good for remote simulation"
                else
                    print_warning "Latency is high. Consider using Ethernet or 5GHz WiFi"
                fi

                # Save Jetson IP for convenience
                echo "# Jetson IP (detected by install script)" >> "$HOME/.bashrc"
                echo "export JETSON_IP=$JETSON_IP" >> "$HOME/.bashrc"
                print_success "Jetson IP saved to .bashrc as \$JETSON_IP"
            else
                print_error "Cannot reach Jetson at $JETSON_IP"
                print_info "Please verify:"
                print_info "  - Both devices are on the same network"
                print_info "  - Jetson IP address is correct"
                print_info "  - Firewall allows ping (ICMP)"
            fi
        fi
    fi

    echo ""
}

################################################################################
# Create Convenience Scripts
################################################################################

create_scripts() {
    print_header "Creating Convenience Scripts"

    SCRIPTS_DIR="$HOME/jetank_sim"
    mkdir -p "$SCRIPTS_DIR"

    # Start Gazebo script
    cat > "$SCRIPTS_DIR/start_gazebo.sh" << 'EOF'
#!/bin/bash
# Quick start script for Gazebo on laptop

export ROS_DOMAIN_ID=42
cd ~/ros2_ws  # Adjust this path to your workspace
source install/setup.bash

echo "Starting Gazebo Fortress for remote simulation..."
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""

ros2 launch jetank_simulation gazebo_remote.launch.py
EOF

    # Start with world selector
    cat > "$SCRIPTS_DIR/start_gazebo_world.sh" << 'EOF'
#!/bin/bash
# Start Gazebo with world selection

export ROS_DOMAIN_ID=42
cd ~/ros2_ws  # Adjust this path to your workspace
source install/setup.bash

WORLD=${1:-empty}

echo "Starting Gazebo with world: $WORLD"
echo "Available worlds: empty, simple_test, obstacle_course, sock_arena"
echo ""

case $WORLD in
    empty)
        ros2 launch jetank_simulation gazebo_remote.launch.py
        ;;
    simple_test|obstacle_course|sock_arena)
        ros2 launch jetank_ros2_main gazebo_sim.launch.py world:=$WORLD
        ;;
    *)
        echo "Unknown world: $WORLD"
        echo "Usage: $0 [empty|simple_test|obstacle_course|sock_arena]"
        exit 1
        ;;
esac
EOF

    # View cameras script
    cat > "$SCRIPTS_DIR/view_cameras.sh" << 'EOF'
#!/bin/bash
# View robot cameras

export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash

echo "Opening camera viewer..."
echo "Select topic: /stereo_camera/left/image_raw or /stereo_camera/right/image_raw"
echo ""

ros2 run rqt_image_view rqt_image_view
EOF

    # Control robot script
    cat > "$SCRIPTS_DIR/control_robot.sh" << 'EOF'
#!/bin/bash
# Control robot with keyboard

export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash

echo "Keyboard control for JeTank"
echo "Make sure Gazebo and robot nodes are running!"
echo ""

ros2 run teleop_twist_keyboard teleop_twist_keyboard
EOF

    # Check connection script
    cat > "$SCRIPTS_DIR/check_connection.sh" << 'EOF'
#!/bin/bash
# Check ROS 2 network connection

export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash

echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""
echo "Available nodes:"
ros2 node list
echo ""
echo "Available topics:"
ros2 topic list | grep -E "cmd_vel|odom|camera" || echo "No robot topics found"
echo ""
echo "If you don't see robot topics, check:"
echo "  - Jetson is running robot_remote.launch.py"
echo "  - ROS_DOMAIN_ID matches on both machines"
echo "  - Firewall allows ROS 2 traffic"
EOF

    # Make all scripts executable
    chmod +x "$SCRIPTS_DIR"/*.sh

    print_success "Created convenience scripts in $SCRIPTS_DIR/"
    echo ""
    echo "  start_gazebo.sh          - Start Gazebo (empty world)"
    echo "  start_gazebo_world.sh    - Start with world selection"
    echo "  view_cameras.sh          - View robot cameras"
    echo "  control_robot.sh         - Control robot with keyboard"
    echo "  check_connection.sh      - Check ROS 2 network"
    echo ""
    print_info "Add to PATH: echo 'export PATH=\$PATH:$SCRIPTS_DIR' >> ~/.bashrc"
    echo ""
}

################################################################################
# Verification
################################################################################

verify_installation() {
    print_header "Verifying Installation"

    source /opt/ros/humble/setup.bash

    # Check ROS 2 commands
    if command -v ros2 &> /dev/null; then
        print_success "ROS 2 command available"
    else
        print_error "ROS 2 command not found"
        return 1
    fi

    # Check Gazebo
    if ros2 pkg list | grep -q "ros_gz_sim"; then
        print_success "Gazebo Fortress packages installed"
    else
        print_error "Gazebo Fortress packages not found"
        return 1
    fi

    # Check environment variables
    if [ -n "$ROS_DOMAIN_ID" ]; then
        print_success "ROS_DOMAIN_ID set to: $ROS_DOMAIN_ID"
    else
        print_warning "ROS_DOMAIN_ID not set in current shell"
        print_info "It will be set after you restart your terminal"
    fi

    print_success "Verification complete"
    echo ""
}

################################################################################
# Main Installation Flow
################################################################################

main() {
    clear

    print_header "JeTank Remote Simulation - Laptop Setup"

    echo "This script will install and configure your laptop for remote simulation"
    echo "with the JeTank robot running on Jetson."
    echo ""
    echo "Configuration:"
    echo "  ROS_DOMAIN_ID: $DOMAIN_ID (set DOMAIN_ID env var to change)"
    echo ""

    if ! prompt_yes_no "Continue with installation?"; then
        echo "Installation cancelled."
        exit 0
    fi

    echo ""

    # Run installation steps
    check_system
    install_dependencies
    configure_network
    configure_firewall
    setup_workspace
    test_network
    create_scripts
    verify_installation

    # Final summary
    print_header "Installation Complete! 🎉"

    echo "Next steps:"
    echo ""
    echo "1. Restart your terminal (or run: source ~/.bashrc)"
    echo ""
    echo "2. On this laptop, start Gazebo:"
    print_info "   ~/jetank_sim/start_gazebo.sh"
    echo ""
    echo "3. On your Jetson, run:"
    print_info "   ros2 launch jetank_simulation robot_remote.launch.py"
    echo ""
    echo "4. Control the robot:"
    print_info "   ~/jetank_sim/control_robot.sh"
    echo ""
    echo "For detailed instructions, see:"
    print_info "   REMOTE_SIMULATION_GUIDE.md"
    echo ""

    print_success "Happy simulating! 🚀"
}

# Run main function
main "$@"
