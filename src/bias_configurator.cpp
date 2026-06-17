#include "bias_configurator.hpp"
#include <metavision/sdk/stream/camera.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <iostream>

bool oa::BiasConfigurator::apply(Metavision::Camera& camera) {
    try {
        Metavision::I_LL_Biases* biases =
            camera.get_device().get_facility<Metavision::I_LL_Biases>();

        if (biases == nullptr) {
            std::cerr << "[OSTRZEŻENIE] Nie znaleziono interfejsu biases w tym urządzeniu.\n";
            return false;
        }

        std::cout << "-> Wykryto sensor GenX320. Konfiguracja czułości...\n";
        biases->set("bias_diff_on",  DIFF_ON_VALUE);
        biases->set("bias_diff_off", DIFF_OFF_VALUE);
        std::cout << "-> Parametry Biases (" << DIFF_ON_VALUE << "/" << DIFF_OFF_VALUE
                  << ") zaaplikowane pomyślnie!\n";
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[BŁĄD BIASES] Nie udało się skonfigurować rejestrów sprzętowych: "
                  << e.what() << "\n";
        return false;
    }
}
