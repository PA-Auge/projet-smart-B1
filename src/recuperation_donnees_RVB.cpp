#include "Adafruit_TCS34725.h"
#include <EEPROM.h>

// ─── Constantes ──────────────────────────────────────────────────────────────

const int INTERVAL_MS      = 500;
const int BYTES_PER_RECORD = 8;                            // 4 × uint16_t (R, G, B, C)
const int MAX_RECORDS      = EEPROM.length() / BYTES_PER_RECORD; // 128 sur Uno

// ─── Objets ──────────────────────────────────────────────────────────────────

Adafruit_TCS34725 RGB_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// ─── Variables globales ──────────────────────────────────────────────────────

int      record_count = 0;
uint32_t last_time    = 0;
bool     full         = false;

// ─── Fonctions EEPROM ────────────────────────────────────────────────────────

// Écrit un uint16_t sur 2 octets à l'adresse donnée
void eeprom_write_u16(int address, uint16_t value) {
    EEPROM.write(address,     (value >> 8) & 0xFF);  // octet haut
    EEPROM.write(address + 1,  value       & 0xFF);  // octet bas
}

// Lit un uint16_t sur 2 octets à l'adresse donnée
uint16_t eeprom_read_u16(int address) {
    return ((uint16_t)EEPROM.read(address) << 8) | EEPROM.read(address + 1);
}

// Sauvegarde une mesure RGBC à l'index donné
void save_record(int index, uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    int addr = index * BYTES_PER_RECORD;
    eeprom_write_u16(addr,     r);
    eeprom_write_u16(addr + 2, g);
    eeprom_write_u16(addr + 4, b);
    eeprom_write_u16(addr + 6, c);
}

// Lit et affiche toutes les mesures stockées via Serial
void dump_records() {
    Serial.println("=== Lecture EEPROM ===");
    Serial.print("Nombre de mesures : ");
    Serial.println(record_count);
    Serial.println("Index | R     | G     | B     | C");

    for (int i = 0; i < record_count; i++) {
        int addr = i * BYTES_PER_RECORD;
        uint16_t r = eeprom_read_u16(addr);
        uint16_t g = eeprom_read_u16(addr + 2);
        uint16_t b = eeprom_read_u16(addr + 4);
        uint16_t c = eeprom_read_u16(addr + 6);

        Serial.print(i);      Serial.print(" | ");
        Serial.print(r);      Serial.print(" | ");
        Serial.print(g);      Serial.print(" | ");
        Serial.print(b);      Serial.print(" | ");
        Serial.println(c);
    }
    Serial.println("=== Fin ===");
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);

    if (!RGB_sensor.begin()) {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Capteur RVB non détecté !");
        while (1);  // bloque si pas de capteur
    }

    Serial.println("Capteur RVB prêt.");
    Serial.print("Capacité max : ");
    Serial.print(MAX_RECORDS);
    Serial.println(" mesures");

    // Si connecté à l'ordi au démarrage → afficher les données stockées
    dump_records();
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop() {
    if (full) {
        // EEPROM pleine : LED clignote comme avertissement
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(200);
        return;
    }

    if (millis() - last_time >= INTERVAL_MS) {
        last_time = millis();

        uint16_t r, g, b, c;
        RGB_sensor.getRawData(&r, &g, &b, &c);
        save_record(record_count, r, g, b, c);
        record_count++;

        Serial.print("Mesure ");
        Serial.print(record_count);
        Serial.print(" sauvegardée — R:");
        Serial.print(r); Serial.print(" G:");
        Serial.print(g); Serial.print(" B:");
        Serial.print(b); Serial.print(" C:");
        Serial.println(c);

        if (record_count >= MAX_RECORDS) {
            full = true;
            Serial.println("⚠ EEPROM pleine !");
        }
    }
}