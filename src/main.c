#include <stdio.h>
#include <math.h>

// --- Prototypes des fonctions --- 

void robot_dynamics(float tau1, float tau2, float th1, float th2, float dth1, float dth2, float *ddth1, float *ddth2);
float fuzzy_compute(float e_actuel, float de_actuel, float K_E, float K_DE, float K_U);

// Définition de PI pour le calcul des trajectoires
#define PI 3.14159265359f

int main() {
    // --- 1. Initialisation des paramètres de simulation ---
    float Ts = 0.001f;     // Période d'échantillonnage 
    float t_max = 10.0f;   // Durée totale de la simulation 
    float t;

    // Vecteur d'état : x1 = th1, x2 = dth1, x3 = th2, x4 = dth2 
    // On suppose que le robot commence au repos, à la position zéro.
    float x1 = 0.0f, x2 = 0.0f, x3 = 0.0f, x4 = 0.0f;

    // Variables pour les trajectoires, les erreurs et leurs dérivées 
    float th1_d, th2_d;
    float e1, e2;
    float e1_prev = 0.0f, e2_prev = 0.0f;
    float de1, de2;

    // Variables pour les couples (sorties du contrôleur flou) 
    float tau1, tau2;
    // Variables pour les accélérations (sorties du modèle dynamique) 
    float ddth1, ddth2;

    // --- Gains du contrôleur flou ---
    
    float KE1 = 15.0f, KDE1 = 0.95f, KU1 = 120.0f;
    float KE2 = 8.0f, KDE2 = 0.16f, KU2 = 30.0f;

    // --- Ouverture du fichier texte   ---
    FILE *fp = fopen("data.txt", "w");
    if (fp == NULL) {
        printf("Erreur : Impossible d'ouvrir ou de creer le fichier data.txt\n");
        return 1;
    }

    printf("Simulation en cours...\n");

    // --- 2. Boucle Principale de Simulation ---
    for (t = 0.0f; t <= t_max; t += Ts) {
        
        // A. Génération des trajectoires désirées 
        th1_d = 0.5f * sinf(2.0f * PI * t);
        th2_d = 0.5f * cosf(2.0f * PI * t);

        // B. Calcul des erreurs de poursuite 
        e1 = th1_d - x1;
        e2 = th2_d - x3;

        // Calcul des dérivées de l'erreur (taux de variation) 
        if (t == 0.0f) {
            de1 = 0.0f; 
            de2 = 0.0f;
        } else {
            de1 = (e1 - e1_prev) / Ts;
            de2 = (e2 - e2_prev) / Ts;
        }
        // Mise en mémoire de l'erreur pour l'itération suivante
        e1_prev = e1;
        e2_prev = e2;

        // C. Exécution du contrôleur flou 
        tau1 = fuzzy_compute(e1, de1, KE1, KDE1, KU1);
        tau2 = fuzzy_compute(e2, de2, KE2, KDE2, KU2);

        // D. Calcul de la dynamique du robot 
        // On envoie les couples et l'état actuel pour obtenir les accélérations (ddth1, ddth2)
        robot_dynamics(tau1, tau2, x1, x3, x2, x4, &ddth1, &ddth2);

        // E. Sauvegarde des résultats selon le format exigé 
        fprintf(fp, "%lf %lf %lf %lf %lf\n", (double)t, (double)th1_d, (double)x1, (double)th2_d, (double)x3);

        // F. Discrétisation d'Euler (Mise à jour des états) 
        // Les nouvelles positions et vitesses deviennent les valeurs pour la prochaine itération
        x1 = x1 + Ts * x2;       // Nouvelle position 1
        x2 = x2 + Ts * ddth1;    // Nouvelle vitesse 1
        x3 = x3 + Ts * x4;       // Nouvelle position 2
        x4 = x4 + Ts * ddth2;    // Nouvelle vitesse 2
    }

    // --- Fermeture du fichier ---
    fclose(fp);
    printf("Simulation terminee avec succes. Les donnees sont dans data.txt\n");

    return 0;
}