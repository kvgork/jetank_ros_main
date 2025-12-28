#!/bin/bash
################################################################################
# JeTank Remote Simulation - Jetson Installation Script
#
# This script configures the Jetson for running robot nodes while Gazebo
# runs remotely on a laptop.
#
# Usage:
#   chmod +x install_jetson.sh
#   ./install_jetson.sh
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

    # Check if running on Jetson
    if [ -f /etc/nv_tegra_release ]; then
        JETSON_INFO=$(cat /etc/nv_tegra_release)
        print_success "Running on Jetson: $JETSON_INFO"
    else
        print_warning "Not detected as Jetson device"
        if ! prompt_yes_no "Continue anyway?"; then
            exit 1
        fi
    fi

    # Check OS
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        echo "OS: $NAME $VERSION"
    fi

    # Check if ROS 2 Humble is installed
    if [ -f /opt/ros/humble/setup.bash ]; then
        print_success "ROS 2 Humble found"
    else
        print_error "ROS 2 Humble not found!"
        print_info "Please install ROS 2 Humble first"
        exit 1
    fi

    # Check workspace
    if [ -f "install/setup.bash" ]; then
        print_success "ROS 2 workspace found"
    else
        print_warning "Not in workspace root. Please cd to ~/workspaces/ros2_ws"
    fi

    print_success "System check passed"
    echo ""
}

################################################################################
# Install Dependencies
################################################################################

install_dependencies() {
    print_header "Installing Dependencies"

    print_info "Updating package lists..."
    sudo apt update

    print_info "Installing ROS 2 packages..."

    PACKAGES=(
        "ros-humble-ros-gz-sim"
        "ros-humble-diff-drive-controller"
        "ros-humble-joint-state-broadcaster"
        "ros-humble-controller-manager"
        "ros-humble-xacro"
        "ros-humble-robot-state-publisher"
        "ros-humble-rmw-fastrtps-cpp"
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
            sed -i 's/^export ROS_DOMAIN_ID=/#&/' "$BASHRC"
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

    print_success "Network configuration complete"
    echo ""
}

################################################################################
# Firewall Configuration
################################################################################

configure_firewall() {
    print_header "Configuring Firewall"

    if ! command -v ufw &> /dev/null; then
        print_info "UFW firewall not installed, skipping..."
        return
    fi

    if sudo ufw status | grep -q "Status: active"; then
        print_warning "Firewall is currently active"

        if prompt_yes_no "Open ROS 2 DDS ports?"; then
            print_info "Opening ROS 2 DDS ports..."

            sudo ufw allow 7400:7500/udp comment 'ROS 2 DDS Discovery'
            sudo ufw allow 7400:7500/tcp comment 'ROS 2 DDS Discovery'
            sudo ufw allow 30000:65535/udp comment 'ROS 2 DDS Data'

            print_success "Firewall ports opened"
        else
            print_warning "Firewall not configured"
            print_info "For testing, you may want to: sudo ufw disable"
        fi
    else
        print_success "Firewall is disabled (good for testing)"
    fi

    echo ""
}

################################################################################
# Performance Configuration
################################################################################

configure_performance() {
    print_header "Jetson Performance Configuration"

    # Check if running on actual Jetson hardware
    if command -v jetson_clocks &> /dev/null; then
        print_info "Jetson performance tools detected"

        if prompt_yes_no "Set Jetson to maximum performance mode?"; then
            print_info "Setting power mode to MAXN..."
            sudo nvpmodel -m 0

            print_info "Enabling jetson_clocks..."
            sudo jetson_clocks

            print_success "Jetson set to maximum performance"
            print_info "Monitor with: tegrastats"
        fi
    else
        print_info "Not running on Jetson hardware, skipping performance config"
    fi

    echo ""
}

################################################################################
# Network Test
################################################################################

test_network() {
    print_header "Network Connectivity Test"

    echo "Testing connection to laptop..."
    echo ""

    if prompt_yes_no "Would you like to test connectivity to laptop now?"; then
        read -p "Enter laptop IP address (e.g., 192.168.1.100): " LAPTOP_IP

        if [ -n "$LAPTOP_IP" ]; then
            print_info "Testing connection to $LAPTOP_IP..."

            if ping -c 3 -W 2 "$LAPTOP_IP" &> /dev/null; then
                print_success "Successfully connected to laptop at $LAPTOP_IP"

                # Measure latency
                LATENCY=$(ping -c 10 "$LAPTOP_IP" | tail -1 | awk -F '/' '{print $5}')
                print_info "Average latency: ${LATENCY}ms"

                if (( $(echo "$LATENCY < 50" | bc -l) )); then
                    print_success "Latency is good for remote simulation"
                else
                    print_warning "Latency is high. Consider using Ethernet or 5GHz WiFi"
                fi

                # Save laptop IP
                echo "# Laptop IP (detected by install script)" >> "$HOME/.bashrc"
                echo "export LAPTOP_IP=$LAPTOP_IP" >> "$HOME/.bashrc"
                print_success "Laptop IP saved to .bashrc as \$LAPTOP_IP"
            else
                print_error "Cannot reach laptop at $LAPTOP_IP"
                print_info "Please verify network configuration"
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

    # Start robot nodes script
    cat > "$SCRIPTS_DIR/start_robot.sh" << 'EOF'
#!/bin/bash
# Start robot nodes for remote simulation

export ROS_DOMAIN_ID=42
cd ~/workspaces/ros2_ws
source install/setup.bash

echo "Starting JeTank robot nodes for remote simulation..."
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""
echo "Make sure Gazebo is running on laptop first!"
echo ""

ros2 launch jetank_simulation robot_remote.launch.py
EOF

    # Start local simulation script
    cat > "$SCRIPTS_DIR/start_local_sim.sh" << 'EOF'
#!/bin/bash
# Start complete local simulation (if Jetson is powerful enough)

export ROS_DOMAIN_ID=42
cd ~/workspaces/ros2_ws
source install/setup.bash

echo "Starting local simulation on Jetson..."
echo "(This may be slow - consider using remote simulation instead)"
echo ""

ros2 launch jetank_simulation gazebo.launch.py
EOF

    # Test drive script
    cat > "$SCRIPTS_DIR/test_drive.sh" << 'EOF'
#!/bin/bash
# Run drive test

export ROS_DOMAIN_ID=42
cd ~/workspaces/ros2_ws
source install/setup.bash

echo "Running differential drive test..."
echo ""

ros2 run jetank_ros2_main test_drive
EOF

    # Test cameras script
    cat > "$SCRIPTS_DIR/test_cameras.sh" << 'EOF'
#!/bin/bash
# Run camera test

export ROS_DOMAIN_ID=42
cd ~/workspaces/ros2_ws
source install/setup.bash

echo "Running stereo camera test..."
echo ""

ros2 run jetank_ros2_main test_cameras
EOF

    # Check connection script
    cat > "$SCRIPTS_DIR/check_connection.sh" << 'EOF'
#!/bin/bash
# Check ROS 2 network connection

export ROS_DOMAIN_ID=42
source ~/workspaces/ros2_ws/install/setup.bash

echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""
echo "Available nodes:"
ros2 node list
echo ""
echo "Controllers:"
ros2 control list_controllers
echo ""
echo "If you don't see laptop nodes, check:"
echo "  - Laptop is running gazebo_remote.launch.py"
echo "  - ROS_DOMAIN_ID matches ($ROS_DOMAIN_ID)"
echo "  - Both machines can ping each other"
EOF

    # Performance monitoring script
    cat > "$SCRIPTS_DIR/monitor_performance.sh" << 'EOF'
#!/bin/bash
# Monitor Jetson performance

if command -v tegrastats &> /dev/null; then
    echo "Monitoring Jetson performance..."
    echo "Press Ctrl+C to stop"
    echo ""
    tegrastats
else
    echo "tegrastats not available (not running on Jetson?)"
    echo "Using htop instead..."
    htop
fi
EOF

    # Make all scripts executable
    chmod +x "$SCRIPTS_DIR"/*.sh

    print_success "Created convenience scripts in $SCRIPTS_DIR/"
    echo ""
    echo "  start_robot.sh           - Start robot nodes (remote sim)"
    echo "  start_local_sim.sh       - Start local simulation"
    echo "  test_drive.sh            - Test differential drive"
    echo "  test_cameras.sh          - Test stereo cameras"
    echo "  check_connection.sh      - Check network connection"
    echo "  monitor_performance.sh   - Monitor Jetson performance"
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

    # Check ROS 2
    if command -v ros2 &> /dev/null; then
        print_success "ROS 2 command available"
    else
        print_error "ROS 2 command not found"
        return 1
    fi

    # Check workspace packages
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        if ros2 pkg list | grep -q "jetank_simulation"; then
            print_success "JeTank packages found"
        else
            print_warning "JeTank packages not found - have you built the workspace?"
        fi
    fi

    # Check environment variables
    if [ -n "$ROS_DOMAIN_ID" ]; then
        print_success "ROS_DOMAIN_ID set to: $ROS_DOMAIN_ID"
    else
        print_warning "ROS_DOMAIN_ID not set in current shell"
    fi

    print_success "Verification complete"
    echo ""
}

################################################################################
# Main Installation Flow
################################################################################

main() {
    clear

    print_header "JeTank Remote Simulation - Jetson Setup"

    echo "This script will configure your Jetson for remote simulation"
    echo "with Gazebo running on a laptop."
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
    configure_performance
    test_network
    create_scripts
    verify_installation

    # Final summary
    print_header "Installation Complete! 🎉"

    echo "Next steps:"
    echo ""
    echo "1. Restart your terminal (or run: source ~/.bashrc)"
    echo ""
    echo "2. On your laptop, start Gazebo first"
    echo ""
    echo "3. On this Jetson, start robot nodes:"
    print_info "   ~/jetank_sim/start_robot.sh"
    echo ""
    echo "4. Test the robot:"
    print_info "   ~/jetank_sim/test_drive.sh"
    echo ""
    echo "For detailed instructions, see:"
    print_info "   REMOTE_SIMULATION_GUIDE.md"
    echo ""

    print_success "Happy coding! 🚀"
}

# Run main function
main "$@"
