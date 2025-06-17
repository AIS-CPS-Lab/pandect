#include <event_camera_codecs/event_processor.h>
#include <threepp/threepp.hpp>

class PlotProcessor : event_camera_codecs::EventProcessor {
public:
    PlotProcessor(threepp::InstancedMesh particles)
        : m_particles(particles), m_numParticles(particles.count()) {}

    void eventCD(uint64_t, uint16_t ex, uint16_t ey, uint8_t polarity) override {
        m_matrix.identity();
        m_matrix.setPosition(ex, ey, m_depth);

        m_particles.setMatrixAt(m_currentIndex, m_matrix);
        m_currentIndex++;
        if (m_currentIndex >= m_numParticles) {
            m_currentIndex = 0;
        }
    }

    void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {
        // No action needed for external trigger
    }

    void finished() override {
        m_particles.instanceMatrix.needsUpdate();
        m_depth ++;
    }

private:
    unsigned int m_numParticles;
    threepp::InstancedMesh m_particles;
    threepp::Matrix4 m_matrix;
    unsigned int m_currentIndex = 0;
    unsigned int m_depth = 0;
};