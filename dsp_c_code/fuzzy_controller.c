#include <math.h>

// --- Macros Utilitaires ---
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

/* =========================================================================
 * FONCTIONS DE FORME (Membership Functions)
 * ========================================================================= */
// Fonction Triangle (trimf)
float trimf(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x == b) return 1.0f;
    if (x < b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

// Fonction Trapèze (trapmf)
float trapmf(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) return 0.0f;
    if (x >= b && x <= c) return 1.0f;
    if (x < b) return (x - a) / (b - a);
    return (d - x) / (d - c);
}

/* =========================================================================
 * MOTEUR D'INFÉRENCE FLOU (7 Variables)
 * ========================================================================= */
float fuzzy_compute(float e_actuel, float de_actuel, float K_E, float K_DE, float K_U) 
{
    // --- 1. Normalisation et Saturation ---
    float E  = e_actuel * K_E;
    float DE = de_actuel * K_DE;
    
    // Saturation stricte à [-1, 1]
    if (E > 1.0f) E = 1.0f;
    else if (E < -1.0f) E = -1.0f;
    if (DE > 1.0f) DE = 1.0f;
    else if (DE < -1.0f) DE = -1.0f;

    // --- 2. Fuzzification (7 Variables) ---
    float mu_E[7], mu_DE[7];
    
    // Évaluation pour l'Erreur (E)
    mu_E[0] = trapmf(E, -1.5f, -1.0f, -0.66f, -0.33f); // NB (Negative Big)
    mu_E[1] = trimf(E, -1.0f, -0.66f, -0.33f);         // NM (Negative Medium)
    mu_E[2] = trimf(E, -0.66f, -0.33f, 0.0f);          // NS (Negative Small)
    mu_E[3] = trimf(E, -0.33f,  0.0f, 0.33f);          // Z  (Zero)
    mu_E[4] = trimf(E,  0.0f,  0.33f, 0.66f);          // PS (Positive Small)
    mu_E[5] = trimf(E,  0.33f,  0.66f, 1.0f);          // PM (Positive Medium)
    mu_E[6] = trapmf(E,  0.33f,  0.66f, 1.0f, 1.5f);   // PB (Positive Big)
    
    // Évaluation pour la Dérivée de l'erreur (DE)
    mu_DE[0] = trapmf(DE, -1.5f, -1.0f, -0.66f, -0.33f); // NB
    mu_DE[1] = trimf(DE, -1.0f, -0.66f, -0.33f);         // NM
    mu_DE[2] = trimf(DE, -0.66f, -0.33f, 0.0f);          // NS
    mu_DE[3] = trimf(DE, -0.33f,  0.0f, 0.33f);          // Z
    mu_DE[4] = trimf(DE,  0.0f,  0.33f, 0.66f);          // PS
    mu_DE[5] = trimf(DE,  0.33f,  0.66f, 1.0f);          // PM
    mu_DE[6] = trapmf(DE,  0.33f,  0.66f, 1.0f, 1.5f);   // PB

    // --- 3. Base de Règles (Matrice 7x7 type MacVicar-Whelan) ---
    // Centres de sortie: NB=0, NM=1, NS=2, Z=3, PS=4, PM=5, PB=6
    float centers[7] = {-1.0f, -0.66f, -0.33f, 0.0f, 0.33f, 0.66f, 1.0f};
    
    // Lignes = Erreur (E), Colonnes = Dérivée (DE)
    int rules[7][7] = {
        {0, 0, 0, 0, 1, 2, 3}, // Si E=NB
        {0, 0, 0, 1, 2, 3, 4}, // Si E=NM
        {0, 0, 1, 2, 3, 4, 5}, // Si E=NS
        {0, 1, 2, 3, 4, 5, 6}, // Si E=Z
        {1, 2, 3, 4, 5, 6, 6}, // Si E=PS
        {2, 3, 4, 5, 6, 6, 6}, // Si E=PM
        {3, 4, 5, 6, 6, 6, 6}  // Si E=PB
    };

    // --- 4. Défuzzification (Barycentre) ---
    float num = 0.0f;
    float den = 0.0f;
    int i_e, i_de;
    
    for (i_e = 0; i_e < 7; i_e++) {
        for (i_de = 0; i_de < 7; i_de++) {
            // Application AND (Min)
            float fire = MIN(mu_E[i_e], mu_DE[i_de]);
            
            if (fire > 0.0f) {
                int out_idx = rules[i_e][i_de];
                num += fire * centers[out_idx];
                den += fire;
            }
        }
    }
    
    // --- 5. Sortie et Dénormalisation ---
    float DU_norm = (den != 0.0f) ? (num / den) : 0.0f;
    
    // Application du Gain de sortie Ku
    return DU_norm * K_U;
}