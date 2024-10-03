#!/bin/bash

# Script pour pull les changements de 'origin main'

echo "Pull des dernières modifications depuis 'origin main'..."

# Pull des modifications de la branche 'main' de l'origine
git pull origin main

# Vérification de l'état après le pull
if [ $? -eq 0 ]; then
    echo "Pull réussi."
else
    echo "Erreur lors du pull. Veuillez vérifier l'état du dépôt."
fi

sleep 2
