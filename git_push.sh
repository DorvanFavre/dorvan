#!/bin/bash

# Script pour ajouter, committer et pousser sur la branche 'main' de l'origine

# Vérification des modifications non commitées
if [[ -n $(git status --porcelain) ]]; then
    echo "Des modifications ont été détectées. Ajout de tous les fichiers..."
    
    # Ajoute tous les fichiers modifiés ou nouveaux
    git add *
    
    # Commit avec un message par défaut
    commit_message="Auto-commit on $(date)"
    git commit -m "$commit_message"
    
    # Pousse sur la branche main de l'origine
    git push origin main
    
    echo "Les changements ont été poussés avec succès sur 'origin main'."
else
    echo "Aucune modification détectée."
fi
