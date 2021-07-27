#include <mitsuba/render/emitter.h>
#include <mitsuba/render/shape.h>
#include <mitsuba/render/medium.h>
#include <mitsuba/hw/gpuprogram.h>
#include <mitsuba/core/warp.h>

MTS_NAMESPACE_BEGIN

class CollimatedBeamLight : public Emitter {
public:
    CollimatedBeamLight(const Properties &props) : Emitter(props) {
        
        m_type |= EDeltaDirection;

        m_radius = props.getFloat("radius", 0.01f);
        m_intensity = props.getSpectrum("intensity", Spectrum::getD65());
        configure();
    }

    CollimatedBeamLight(Stream *stream, InstanceManager *manager)
        : Emitter(stream, manager) {
        m_intensity = Spectrum(stream);
        m_radius = stream->readFloat();
        m_power = Spectrum(stream);
        configure();
    }

    void configure() {
        m_surfaceArea = m_radius * m_radius * M_PI;
        m_invSurfaceArea = 1.f / m_surfaceArea;
        m_power = m_intensity * m_surfaceArea;
        m_direction = m_worldTransform->eval(0.f)(Vector3(0.f, 0.f, 1.f));
    }

    void serialize(Stream *stream, InstanceManager *manager) const {
        Emitter::serialize(stream, manager);
        m_intensity.serialize(stream);
        stream->writeFloat(m_radius);
        m_power.serialize(stream);
    }

    // used in bdpt
    Spectrum samplePosition(PositionSamplingRecord &pRec,
            const Point2 &sample, const Point2 *extra) const {
        Point2 pointOnDisk = warp::squareToUniformDiskConcentric(sample) * m_radius;
        pRec.p = m_worldTransform->eval(pRec.time)(Point(pointOnDisk.x, pointOnDisk.y, 0.f));
        pRec.n = m_direction;
        pRec.pdf = m_invSurfaceArea;
        pRec.measure = EDiscrete;
        return m_power;
    }

    // used in bdpt
    Spectrum evalPosition(const PositionSamplingRecord &pRec) const {
        Point local = m_worldTransform->eval(pRec.time).inverse()(pRec.p);
        Vector2 planeProjection = Vector2(local.x, local.y);
        if (pRec.measure != EDiscrete || planeProjection.length() > m_radius || local.z < 0.f )
            return Spectrum(0.f);
        else
            return m_intensity;
    }

    // not used in bdpt
    Spectrum eval(const Intersection &its, const Vector &d) const {
        Point local = m_worldTransform->eval(its.time).inverse()(its.p);
        Vector2 planeProjection = Vector2(local.x, local.y);

        Float dp = dot(its.shFrame.n, d);
        if ( dp < (1.f - Epsilon) ||
             planeProjection.length() > m_radius ||
             local.z < 0.f )
            return Spectrum(0.0f);
        else
            return m_intensity;
    }

    // used in bdpt
    Float pdfPosition(const PositionSamplingRecord &pRec) const {
        Point local = m_worldTransform->eval(pRec.time).inverse()(pRec.p);
        Vector2 planeProjection = Vector2(local.x, local.y);
        if (pRec.measure != EDiscrete ||
                planeProjection.length() > m_radius ||
                 local.z < 0.f )
            return 0.f;
        else
            return m_invSurfaceArea;
    }

    // used in bdpt
    Spectrum sampleDirection(DirectionSamplingRecord &dRec,
            PositionSamplingRecord &pRec,
            const Point2 &sample, const Point2 *extra) const {

        Vector local(0.f, 0.f, 1.f);
        dRec.d = Frame(pRec.n).toWorld(local);
        dRec.pdf = 1.f;
        dRec.measure = EDiscrete;
        return Spectrum(1.0f);
    }

    // not called by bdpt
    Spectrum evalDirection(const DirectionSamplingRecord &dRec,
            const PositionSamplingRecord &pRec) const {
        // const Transform &trafo = m_worldTransform->eval(pRec.time);
        // Vector local = trafo.inverse()(dRec.d);
        Float dp = dot(dRec.d, m_direction);
        if ( dp < 0 )
            dp = 0.0f;

        return Spectrum(INV_PI * dp);
    }

    // used in bdpt
    Float pdfDirection(const DirectionSamplingRecord &dRec,
            const PositionSamplingRecord &pRec) const {
        return (dRec.measure == EDiscrete) ? 1.0f : 0.0f;
    }

    // this function is not called in bdpt
    Spectrum sampleRay(Ray &ray,
            const Point2 &spatialSample,
            const Point2 & /*directionalSample*/,
            Float time) const {
        PositionSamplingRecord pRec(time);
        Point2 pointOnDisk = warp::squareToUniformDiskConcentric(spatialSample) * m_radius;
        Point p = m_worldTransform->eval(pRec.time)(Point(pointOnDisk.x, pointOnDisk.y, 0.f));

        ray.setTime(time);
        ray.setOrigin(p);
        ray.setDirection(m_direction);

        Log(EInfo, "sampleRay output : %s", m_power.toString().c_str());
        return m_power;
    }

    Spectrum sampleDirect(DirectSamplingRecord &dRec,
            const Point2 &sample) const {
        
        const Transform &trafo = m_worldTransform->eval(dRec.time);

        // check if dRec.ref lies within the disk area to receive any light
        Point refInLocal = trafo.inverse()(dRec.ref);
        Vector2 planeProjection = Vector2(refInLocal.x, refInLocal.y);
        if (planeProjection.length() > m_radius || refInLocal.z < 0.f ) {
            dRec.pdf = 0.f;
            return Spectrum(0.f);
        }

        Point local(refInLocal.x, refInLocal.y, 0.f);

        dRec.p = trafo(local);
        dRec.d = -m_direction;
        dRec.n = Normal(m_direction);  // unclear if this should be filled
        dRec.dist = refInLocal.z;
        dRec.pdf = 1.0f;
        dRec.measure = EDiscrete;
        
        return m_intensity;
    }

    Float pdfDirect(const DirectSamplingRecord &dRec) const {
        return dRec.measure == EDiscrete ? m_invSurfaceArea : 0.0f;
    }

    AABB getAABB() const {
        return m_worldTransform->getTranslationBounds();
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "CollimatedBeamLight[" << endl
            << "  intensity = " << m_intensity.toString() << "," << endl
            << "  surfaceArea = " << m_surfaceArea << endl
            << "]";
        return oss.str();
    }

    Shader *createShader(Renderer *renderer) const;

    MTS_DECLARE_CLASS()
protected:
    Spectrum m_intensity, m_power;
    Float m_radius, m_surfaceArea, m_invSurfaceArea;
    Vector3 m_direction;

};

// ================ Hardware shader implementation ================

class CollimatedBeamLightShader : public Shader {
public:
    CollimatedBeamLightShader(Renderer *renderer, const Spectrum &intensity)
        : Shader(renderer, EEmitterShader), m_intensity(intensity) {
    }

    void resolve(const GPUProgram *program, const std::string &evalName,
            std::vector<int> &parameterIDs) const {
        parameterIDs.push_back(program->getParameterID(evalName + "_intensity", false));
    }

    void generateCode(std::ostringstream &oss, const std::string &evalName,
            const std::vector<std::string> &depNames) const {
        oss << "uniform vec3 " << evalName << "_intensity;" << endl
            << endl
            << "vec3 " << evalName << "_area(vec2 uv) {" << endl
            << "    return " << evalName << "_intensity * pi;" << endl
            << "}" << endl
            << endl
            << "vec3 " << evalName << "_dir(vec3 wo) {" << endl
            << "    if (cosTheta(wo) < 0.0)" << endl
            << "        return vec3(0.0);" << endl
            << "    return vec3(inv_pi);" << endl
            << "}" << endl;
    }

    void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
        int &textureUnitOffset) const {
        program->setParameter(parameterIDs[0], m_intensity);
    }

    MTS_DECLARE_CLASS()
private:
    Spectrum m_intensity; //, m_power;
    // Float m_radius, m_surfaceArea, m_invSurfaceArea;
    // Vector3 m_direction;
};

Shader *CollimatedBeamLight::createShader(Renderer *renderer) const {
    return new CollimatedBeamLightShader(renderer, m_intensity);
}

MTS_IMPLEMENT_CLASS(CollimatedBeamLightShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(CollimatedBeamLight, false, Emitter)
MTS_EXPORT_PLUGIN(CollimatedBeamLight, "Collimated Beam light");
MTS_NAMESPACE_END