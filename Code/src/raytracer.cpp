#include "raytracer.h"
#include "basic.h"
#define SOFT_SHADOWS_SAMPLES 10

void Raytracer::render(const Scene &scene, Frame *output) {
    // Crée le z_buffer.
    // NOTE: zbuffer est un pointeur vers un array de doubles de taille
    // resolution[0] * resolution[1], aka le nombre de pixels dans l'image.

    int resolutionX = scene.resolution[0];
    int resolutionY = scene.resolution[1];

    double *z_buffer = new double[resolutionX * resolutionY];
    for (int i = 0; i < resolutionX * resolutionY; i++) {
        z_buffer[i] = scene.camera.z_far; // Anciennement DBL_MAX. À remplacer avec la valeur de scene.camera.z_far
    }

    // Calcule les paramètres de la caméra pour les rayons.
    double top = tan(deg2rad(scene.camera.fovy) / 2.0) * std::fabs(scene.camera.z_near);
    double right = top * resolutionX / resolutionY;
    double bottom = -top;
    double left = -right;

    // @@@@@@ VOTRE CODE ICI
    // Calculez les paramètres de la caméra pour les rayons.

    std::ofstream file;
    file.open("output.txt");

    // Itère sur tous les pixels de l'image.
    for (int y = 0; y < resolutionY; y++) {
        if (y % 40) {
            std::cout << "\rScanlines completed: " << y << "/" << resolutionY << '\r';
        }

        for (int x = 0; x < resolutionX; x++) {
            int avg_z_depth = 0;
            double3 avg_ray_color{0, 0, 0};

            for (int iray = 0; iray < scene.samples_per_pixel; iray++) {
                // Génère le rayon approprié pour ce pixel.
                Ray ray;
                // Initialise la profondeur de récursivité du rayon.
                int ray_depth = 0;
                double3 ray_color{0, 0, 0}; // Initialise la couleur du rayon

                double out_z_depth = scene.camera.z_far;

                // @@@@@@ VOTRE CODE ICI
                // Mettez en place le rayon primaire en utilisant les paramètres de la caméra.
                // Lancez le rayon de manière uniformément aléatoire à l'intérieur du pixel
                // dans la zone délimitée par jitter_radius.
                // Faites la moyenne des différentes couleurs obtenues suite à la récursion.

                /*
                 * Les formules suivantes ont été obtenues via un cours d'infographie
                 * donné par une université de l'état d'Ohio:
                 * https://web.cse.ohio-state.edu/~shen.94/681/Site/Slides_files/basic_algo.pdf
                 */

                double3 n = normalize(scene.camera.position - scene.camera.center);
                double3 u = normalize(cross(scene.camera.up, n));
                double3 v = cross(n, u);

                double H = 2.0 * scene.camera.z_near * std::tan(deg2rad(scene.camera.fovy / 2.0));
                double W = H * scene.camera.aspect;
                double3 C = scene.camera.position - n * scene.camera.z_near;
                double3 L = C - u * W / 2.0 - v * H / 2.0;

                double3 pixelPosition = L + u * x * W / double(resolutionX) +
                                        v * y * H / double(resolutionY);

                ray.origin = scene.camera.position;
                ray.direction = normalize(pixelPosition - ray.origin);
                trace(scene, ray, ray_depth, &ray_color, &out_z_depth);
                avg_z_depth += out_z_depth;
                avg_ray_color += ray_color;
            }

            avg_z_depth = avg_z_depth / scene.samples_per_pixel;
            avg_ray_color = avg_ray_color / scene.samples_per_pixel;
            output->set_color_pixel(x, y, avg_ray_color);
            // Test de profondeur
            if (avg_z_depth >= scene.camera.z_near && avg_z_depth <= scene.camera.z_far &&
                avg_z_depth < z_buffer[x + y * resolutionX]) {
                z_buffer[x + y * resolutionX] = avg_z_depth;
                // Met à jour la couleur de l'image (et sa profondeur)
                output->set_depth_pixel(x, y,(avg_z_depth - scene.camera.z_near) / (scene.camera.z_far - scene.camera.z_near));
            }
            file << avg_ray_color.x << " " << avg_ray_color.y << " " << avg_ray_color.z << std::endl;
        }
    }
    file.close();
    delete[] z_buffer;
}

// 3. raytracer.cpp - Raytracer::trace
// Mettez à jour les couleurs et le buffer de sortie s'il y a intersection
// en fonction de raytracer.cpp - Raytracer::shade.
// (Ignorez le lancer de rayons secondaires pour la réflexion et la réfraction pour le moment.
// Laissez raytracer.cpp - Raytracer::shade vide pour le moment).

// @@@@@@ VOTRE CODE ICI
// Veuillez remplir les objectifs suivants:
//   - Déterminer si le rayon intersecte la géométrie.
//   - Calculer la contribution associée à la réflexion.
//   - Calculer la contribution associée à la réfraction.
//   - Mettre à jour la couleur avec le shading +
//     Ajouter réflexion selon material.reflection +
//     Ajouter réfraction selon material.refraction pour la couleur de sortie.
//   - Mettre à jour la nouvelle profondeur.

void Raytracer::trace(const Scene &scene, Ray ray, int ray_depth,
                      double3 *out_color, double *out_z_depth) {
    Intersection hit;

    // Fait appel à l'un des containers spécifiés.
    if (scene.container->intersect(ray, EPSILON, *out_z_depth, &hit, true)) {
        Material &material = ResourceManager::Instance()->materials[hit.key_material];

        // @@@@@@ VOTRE CODE ICI
        // Déterminer la couleur associée à la réflexion d'un rayon de manière récursive.

        double3 reflectedColor;
        double3 refractedColor;
        bool isRefract = false;
        bool isReflex = false;

        // k_reflection is the coefficient of color captured by the object when throwing rays.
        if (material.k_reflection > 0) {
            if (ray_depth < scene.max_ray_depth) {
                isReflex = true;
                // Direction du rayon réfléchi
                double3 d = ray.direction;
                double3 n = hit.normal;
                double3 r = normalize(d - 2 * dot(d, n) * n);

                Ray reflectedRay(hit.position + EPSILON * r, r);
                trace(scene, reflectedRay, ray_depth + 1, &reflectedColor, out_z_depth);
            }
        } else if (material.k_refraction > 0) {
            // On a une réfraction
            double indiceIncident = 1.0; // On suppose qu'on passe toujours de l'air vers autre chose
            double indiceRefractant = material.refractive_index;
            double eta = indiceIncident / indiceRefractant;
            if (ray_depth < scene.max_ray_depth) {
                isRefract = true;
                double3 normal = hit.normal;
                double3 rayDirection = ray.direction;

                double c1 = dot(-rayDirection, normal);
                double c2 = sqrt(1 - pow(eta, 2) * (1 - pow(c1, 2)));

                double3 T = eta * rayDirection + (eta * c1 - c2) * normal;

                Ray refractedRay;
                refractedRay.direction = normalize(T);
                refractedRay.origin = hit.position + EPSILON * T;

                trace(scene, refractedRay, ray_depth + 1, &refractedColor, out_z_depth);
            }
        }

        if (isReflex) {
            reflectedColor *= material.k_reflection;
        }
        if (isRefract) {
            refractedColor *= material.k_refraction;
        }

        double3 backGroundColor = newShade(scene, hit) + reflectedColor + refractedColor;

        if (hit.hitGrid) {
            *out_color = backGroundColor * (hit.transmittance) + hit.scatter;
        } else {
            *out_color = backGroundColor;
        }

        *out_z_depth = hit.depth;
    } else {
        // Si pas d'intersection, on définit la couleur
        *out_color = double3(0, 0, 0);
    }
}

// Small function to keep the code clean
double3 UVMap(double u, double v, const bitmap_image &texture) {
    double3 out_color(0, 0, 0);
    rgb_t colour;

    texture.get_pixel(u, v, colour);

    double r = static_cast<double>(colour.red);
    double g = static_cast<double>(colour.green);
    double b = static_cast<double>(colour.blue);

    out_color = double3(r / 255.0, g / 255.0, b / 255.0);

    return out_color;
}

double3 Raytracer::shade(const Scene &scene, Intersection hit) {
    Material &material = ResourceManager::Instance()->materials[hit.key_material];

    auto k_a = material.k_ambient;
    auto k_d = material.k_diffuse;
    auto k_s = material.k_specular;
    auto normal = hit.normal;
    auto hitPosition = hit.position;
    auto m = material.metallic;
    auto n = material.shininess;
    auto cameraPosition = scene.camera.position;
    double3 color(0, 0, 0);
    double3 colorAlbedo;

    // Si aucune texture n'est présente
    if (material.texture_albedo.height() == 0 ||
        material.texture_albedo.width() == 0) {
        colorAlbedo = material.color_albedo;
    } else {
        auto x = hit.uv.x * material.texture_albedo.width();
        auto y = hit.uv.y * material.texture_albedo.height();
        colorAlbedo = UVMap(x, y, material.texture_albedo);
    }

    std::vector<SphericalLight> lights = scene.lights;
    double3 ambiantLight = scene.ambient_light;

    double3 ambiantContribution = ambiantLight * k_a * colorAlbedo;

    double3 diffuseContribution(0, 0, 0);
    double3 blinnContribution(0, 0, 0);

    for (int i = 0; i < lights.size(); i++) {
        SphericalLight light = lights[i];
        double3 toLight = light.position - hitPosition;
        double lightDistance = length(toLight);
        toLight = normalize(toLight);

        double3 shadowOrigin = hitPosition + EPSILON * hit.normal;
        Ray shadowRay = Ray(shadowOrigin, toLight);

        Intersection shadowHit;
        double lightReceived = 1.0;

        // Handle soft shadows if the light has a radius
        if (light.radius > 0) {
            double visibleRays = 0;
            double3 diskNormal = normalize(hit.position - light.position);
            double3 diskCenter = light.position;

            for (int i = 0; i < SOFT_SHADOWS_SAMPLES; ++i) {
                double2 randomPointOnDisk = random_in_unit_disk() * light.radius;

                double3 firstDiskDirectorVector = normalize(
                        abs(diskNormal.x) > abs(diskNormal.y)
                        ? double3(-diskNormal.z, 0, diskNormal.x)
                        : double3(0, -diskNormal.z, diskNormal.y));
                double3 secondDiskDirectorVector =
                        normalize(cross(diskNormal, firstDiskDirectorVector));

                double3 randomPoint3D = diskCenter +
                                        randomPointOnDisk.x * firstDiskDirectorVector +
                                        randomPointOnDisk.y * secondDiskDirectorVector;

                double3 shadowOrigin = hit.position + EPSILON * hit.normal;
                double3 shadowRayDir = normalize(randomPoint3D - hit.position);
                double distanceToLight = length(randomPoint3D - hit.position);
                Ray shadowRay = Ray(shadowOrigin, shadowRayDir);

                if (!scene.container->intersect(shadowRay, EPSILON, distanceToLight,
                                                &shadowHit, false)) {
                    visibleRays++;
                }
            }
            lightReceived = visibleRays / SOFT_SHADOWS_SAMPLES;
        }

        if (light.radius == 0) {
            if (scene.container->intersect(shadowRay, EPSILON, lightDistance,
                                           &shadowHit, false)) {
                if (shadowHit.depth < lightDistance) {
                    continue;
                }
            }
        }

        if (lightReceived == 0) {
            continue;
        }

        // Si la lumière n'est pas occultée, on calcule la contribution de la lumière au pixel
        /*
         * Lumière ponctuelle
         */

        double nDotL = std::max(dot(normal, toLight), 0.0);
        double3 diffuseContributionWithoutShadow =
                (colorAlbedo * light.emission * k_d * nDotL) / pow(lightDistance, 2);

        double3 bisector = normalize(toLight + normalize(cameraPosition - hitPosition));
        double nDotH = std::max(dot(normal, bisector), 0.0);
        double3 blinnContributionWithoutShadow =
                (m * colorAlbedo + (1 - m)) * (k_s * light.emission * pow(nDotH, n)) /
                pow(lightDistance, 2);

        if(light.radius == 0) {
            diffuseContribution += diffuseContributionWithoutShadow;
            blinnContribution += blinnContributionWithoutShadow;
        } else {
            diffuseContribution += diffuseContributionWithoutShadow * lightReceived;
            blinnContribution += blinnContributionWithoutShadow * lightReceived;
        }
    }

    return ambiantContribution + diffuseContribution + blinnContribution;
}

double3 Raytracer::newShade(const Scene &scene, Intersection hit) {
    Material &material = ResourceManager::Instance()->materials[hit.key_material];

    auto k_a = material.k_ambient;
    auto k_d = material.k_diffuse;
    auto k_s = material.k_specular;
    auto normal = hit.normal;
    auto hitPosition = hit.position;
    auto m = material.metallic;
    auto n = material.shininess;
    auto cameraPosition = scene.camera.position;
    double3 color(0, 0, 0);
    double3 colorAlbedo;
    auto k_reflection = material.k_reflection;

    // Si aucune texture n'est présente
    if (material.texture_albedo.height() == 0 ||
        material.texture_albedo.width() == 0) {
        colorAlbedo = material.color_albedo;
    } else {
        auto x = hit.uv.x * material.texture_albedo.width();
        auto y = hit.uv.y * material.texture_albedo.height();
        colorAlbedo = UVMap(x, y, material.texture_albedo);
    }

    std::vector<SphericalLight> lights = scene.lights;
    double3 ambiantLight = scene.ambient_light;

    color = colorAlbedo * ambiantLight * k_a;

    double3 diffuseContribution(0, 0, 0);
    double3 blinnContribution(0, 0, 0);

    for (int i = 0; i < lights.size(); i++) {
        SphericalLight light = lights[i];

        double3 toLight = light.position - hit.position;
        double lightDistance = length(toLight);
        toLight = normalize(toLight);
        Intersection shadowHit;
        double lightBlocked;
        double3 lightIntensity = light.emission;

        // Si la lumière est un point dans l'espace
        if (light.radius == 0) {
            Ray shadowRay =
                    Ray(hit.position + EPSILON * hit.normal, normalize(light.position - hit.position));
            if (!scene.container->intersect(shadowRay, EPSILON, lightDistance,
                                            &shadowHit, true)) {
                if (shadowHit.hitGrid) {
                    lightBlocked = shadowHit.transmittance;
                    lightIntensity = lightBlocked * light.emission;
                } else {
                    lightBlocked = 0;
                    lightIntensity = (1 - lightBlocked) * light.emission;
                }
            } else {
                lightBlocked = 1.0;
                lightIntensity = (1 - lightBlocked) * light.emission;
            }

            // Évaluer le modèle d'éclairage
            double nDotL = std::max(dot(normal, toLight), 0.0);
            diffuseContribution +=
                    (colorAlbedo * lightIntensity * k_d * nDotL) / pow(lightDistance, 2);

            double3 bisector =
                    normalize(toLight + normalize(cameraPosition - hitPosition));
            double nDotH = std::max(dot(normal, bisector), 0.0);
            blinnContribution +=
                    (m * colorAlbedo + (1 - m)) *
                    (k_s * lightIntensity * pow(nDotH, n)) / pow(lightDistance, 2);
        }
        else {
            // Si la lumière est une sphère avec un rayon non nul
            double3 diskNormal = normalize(hit.position - light.position);
            double3 diskCenter = light.position;
            double3 diffuseContributionWithoutShadow(0, 0, 0);
            double3 blinnContributionWithoutShadow(0, 0, 0);

            double qtyLightHitting = 0;
            for (int j = 0; j < SOFT_SHADOWS_SAMPLES; j++) {
                double2 randomPointOnDisk = random_in_unit_disk() * light.radius;
                double3 firstDiskDirectorVector = normalize(
                        abs(diskNormal.x) > abs(diskNormal.y)
                        ? double3(-diskNormal.z, 0, diskNormal.x)
                        : double3(0, -diskNormal.z, diskNormal.y));
                double3 secondDiskDirectorVector =
                        normalize(cross(diskNormal, firstDiskDirectorVector));
                double3 randomPoint3D = diskCenter +
                                        randomPointOnDisk.x * firstDiskDirectorVector +
                                        randomPointOnDisk.y * secondDiskDirectorVector;
                double3 shadowRayDir = normalize(randomPoint3D - hit.position);
                double distanceToLight = length(randomPoint3D - hit.position);
                Ray shadowRay = Ray(hit.position + EPSILON * hit.normal, shadowRayDir);

                if(!scene.container->intersect(shadowRay, EPSILON, distanceToLight, &shadowHit, true)){
                    if(shadowHit.hitGrid){
                        qtyLightHitting += shadowHit.transmittance;
                    } else {
                        qtyLightHitting += 1;
                    }
                }
                else {
                    qtyLightHitting += 0;
                }
            }
            double averageLightHitting = qtyLightHitting / SOFT_SHADOWS_SAMPLES;
            lightIntensity = (averageLightHitting) * light.emission;

            double nDotL = std::max(dot(normal, toLight), 0.0);
            diffuseContribution += (colorAlbedo * lightIntensity * k_d * nDotL) / pow(lightDistance, 2);

            double3 bisector = normalize(toLight + normalize(cameraPosition - hitPosition));
            double nDotH = std::max(dot(normal, bisector), 0.0);
            blinnContribution += (m * colorAlbedo + (1 - m)) * (k_s * lightIntensity * pow(nDotH, n)) / pow(lightDistance, 2);
        }
    }

    color += diffuseContribution + blinnContribution;

    return color;
}
