#!/bin/bash

echo "Setting up environment..."
{
    sed -i 's/\r$//' setup/setup.sh
    bash -i setup/setup.sh --no_import
} &

echo "Installing pre-commit..."
{
    pre-commit install
    # pre-commit autoupdate
}

echo "Setup complete"
