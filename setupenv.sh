    #!/usr/bin/env bash

    # Exit immediately if a command exits with a non-zero status
    set -e

    # Directory for the virtual environment
    VENV_DIR=".venv"

    echo "Setting up Python virtual environment..."

    # Check for Python 3
    if ! command -v python3 &> /dev/null; then
        echo "Error: python3 is not installed or not in PATH"
        exit 1
    fi

    # Create the virtual environment if it doesn't exist
    if [ ! -d "$VENV_DIR" ]; then
        python3 -m venv "$VENV_DIR"
        echo "Virtual environment created in $VENV_DIR"
    else
        echo "Virtual environment already exists in $VENV_DIR"
    fi

    # Activate the virtual environment
    source "$VENV_DIR/bin/activate"

    # Upgrade pip
    echo "Upgrading pip..."
    pip install --upgrade pip

    # Install requirements if requirements.txt exists
    if [ -f "requirements.txt" ]; then
        echo "Installing requirements from requirements.txt..."
        pip install -r requirements.txt
    else
        echo "No requirements.txt found. Skipping dependency installation."
    fi

    echo ""
    echo "========================================="
    echo "Virtual environment setup complete."
    echo "To activate the environment in your current shell, run:"
    echo "source $VENV_DIR/bin/activate"
    echo "========================================="


