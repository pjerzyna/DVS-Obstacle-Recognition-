#include "bias_configurator.hpp"
#include <metavision/sdk/stream/camera.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <iostream>

void oa::BiasConfigurator::apply(Metavision::Camera& camera) {
    try {
        // Pobieramy dostęp do interfejsu niskopoziomowego urządzenia HAL
        Metavision::I_LL_Biases* biases = camera.get_device().get_facility<Metavision::I_LL_Biases>();

        if (biases != nullptr) {
            std::cout << "-> Wykryto sensor GenX320 w C++. Konfiguracja czułości..." << std::endl;

            // Nadpisujemy parametry sprzętowe komparatorów (dokładnie tak jak w Pythonie)
            biases->set("bias_diff_on", 45);
            biases->set("bias_diff_off", 45);

            std::cout << "-> Parametry Biases (45/45) zaaplikowane pomyślnie!" << std::endl;
        } else {
            std::cerr << "[OSTRZEŻENIE] Nie znaleziono interfejsu biases w tym urządzeniu." << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[BŁĄD BIASES] Nie udało się skonfigurować rejestrów sprzętowych: " << e.what() << std::endl;
    }
}
