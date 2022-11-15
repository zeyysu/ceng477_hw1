#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>

typedef unsigned char RGB[3];

typedef struct
{
	parser::Vec3f o,d;
} ray;


parser::Vec3f multS(parser::Vec3f a,float s)
{
	parser::Vec3f result;
	result.x = a.x*s;
	result.y = a.y*s;
	result.z = a.z*s;
	return result;
}

parser::Vec3f add(parser::Vec3f a, parser::Vec3f b)
{
	parser::Vec3f result;
	result.x = a.x+b.x;
	result.y = a.y+b.y;
	result.z = a.z+b.z;
	return result;
}


parser::Vec3f crossP(parser::Vec3f a,parser::Vec3f b)
{
    parser::Vec3f result;
    result.x = a.y*b.z-a.z*b.y;
    result.y = a.z*b.x-a.x*b.z;
    result.z = a.x*b.y-a.y*b.x;
    return result;
}

parser::Vec3f sub(parser::Vec3f a, parser::Vec3f b)
{
    parser::Vec3f c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;
    return c;
}
float dotP(parser::Vec3f a, parser::Vec3f b)
{
	return (a.x * b.x + a.y * b.y + a.z * b.z);
}
parser::Vec3f normalize(parser::Vec3f a)
{
	return multS(a,1.0/sqrt(dotP(a,a)));
}


float intersectSphere(ray r, parser::Sphere sp, parser::Scene* scene)
{
	float A,B,C; //constants for the quadratic equation
	
	float discriminant;
	
	parser::Vec3f c = scene->vertex_data[sp.center_vertex_id-1];
    
	
	float t,t1,t2;
	

    C = dotP(sub(r.o,c),sub(r.o,c))-sp.radius*sp.radius;

    B= 2* dotP(r.d,sub(r.o,c));

	A= dotP(r.d,r.d);
    
	
	discriminant = B*B-4*A*C;
	
	if (discriminant<0) return -1;
	else if (discriminant==0)
	{
		t = -B / (2*A);
	}
	else
	{
		discriminant = sqrt(discriminant);
		A = 2*A;
		t1 = (-B + discriminant) / A;
		t2 = (-B - discriminant) / A;
		if (t1<t2) t=t1; else t=t2;
	}
	
	return t;
}

float intersectTriangle(ray r, parser::Face tr, parser::Scene *scene)
{
    parser::Vec3f a = scene->vertex_data[tr.v0_id-1];
    parser::Vec3f b = scene->vertex_data[tr.v1_id-1];
    parser::Vec3f c = scene->vertex_data[tr.v2_id-1];
    parser::Vec3f e1 = sub(b,a);
    parser::Vec3f e2 = sub(c,a);
    parser::Vec3f s = sub(r.o,a);
    parser::Vec3f s1 = crossP(r.d,e2);
    parser::Vec3f s2 = crossP(s,e1);
    float t = dotP(s2,e2)/dotP(s1,e1);
    float beta = dotP(s1,s)/dotP(s1,e1);
    float gamma = dotP(s2,r.d)/dotP(s1,e1);
    if (t>0 && beta>=0 && gamma>=0 && beta+gamma<1)
        return t;
    else
        return -1;
}




ray generateRay(parser::Camera camera, int i, int j)
{
    
	ray result;
	float su,sv;
	parser::Vec3f m,q,s;
    int nx=camera.image_width;
    int ny=camera.image_height;
	float right= camera.near_plane.y;
    float left =camera.near_plane.x;
    float top = camera.near_plane.w;
    float bottom = camera.near_plane.z;
    float dist = camera.near_distance;
    parser::Vec3f v = camera.up;
    parser::Vec3f u = crossP(v,multS(camera.gaze,-1.0));

	su = (i+0.5)*(right-left)/nx;
	sv = (j+0.5)*(top-bottom)/ny;
	
	m = add(camera.position,multS(camera.gaze,dist));
	
	q = add(m,add(multS(u,left),multS(v,top)));
	
	s = add(q,add(multS(u,su),multS(v,-sv)));
	
	result.o = camera.position;
	result.d = add(s,multS(camera.position,-1));
	
	return result;
}




parser::Vec3f computeColor(ray r,parser::Scene *scene, int recursionDepth){
    int asd = 0;
    int i;
    int faceI=-1;
	parser::Vec3f c;
	float minT = INT32_MAX; // some large number
	float t;
	parser::Vec3f L,N;
	parser::Vec3f P;
	int minI=-1;
    int c_materialId=-1;
    int n=0;//1 if sphere 2 if triangle


    c.x=c.y=c.z=0;
            if (recursionDepth == -1) {
        return c;
    }

	for (i=0;i<scene->spheres.size();i++){
		t = intersectSphere(r,scene->spheres[i],scene);
        
		if (t<minT && t>=0)
		{
			minI = i;
			minT = t;
            c_materialId = scene->spheres[minI].material_id;
            n= 1;



		}
	}
    for(i=0;i<scene->triangles.size();i++){
       
       
        t = intersectTriangle(r,scene->triangles[i].indices,scene);
        if (t<minT && t>=0)
        {
            
                        
            minI = i;
            minT = t;
            c_materialId = scene->triangles[minI].material_id;
            n=2;
        }
    }
    int meshsize = scene->meshes.size();
    int facesize;
    for(i=0;i<meshsize;i++){
        
        facesize=scene->meshes[i].faces.size();
        for(int j=0;j<facesize;j++){
           
            t = intersectTriangle(r,scene->meshes[i].faces[j],scene);
            if (t<minT && t>=0)
            {
                
                minI = i;
                minT = t;
                faceI=j;
                c_materialId = scene->meshes[minI].material_id;
                n=3;
                
            }
        }
    }
    if(minT== -1){
       
        return c;
    }

    
    if(minI>=0){
        

        int noOfLights = scene->point_lights.size();
        int noOfSpheres = scene->spheres.size();
        int noOfTriangles = scene->triangles.size();
        int noOfMeshes = scene->meshes.size();

        c.x = scene->materials[c_materialId-1].ambient.x * scene->ambient_light.x;
        c.y = scene->materials[c_materialId-1].ambient.y * scene->ambient_light.y;
        c.z = scene->materials[c_materialId-1].ambient.z * scene->ambient_light.z;

        parser::Vec3f intersectionPoint= add(r.o,multS(r.d,minT));
        parser::Vec3f Norm;
        if (n==1){//SPHERE

            parser::Vec3f centerSphere = scene->vertex_data[scene->spheres[minI].center_vertex_id-1];
            Norm = sub(intersectionPoint,centerSphere);
            Norm = normalize(Norm);
        }

        else if (n==2){//TRIANGLE OR MESH
            parser::Vec3f a1 = scene->vertex_data[scene->triangles[minI].indices.v0_id-1];
            parser::Vec3f b1 = scene->vertex_data[scene->triangles[minI].indices.v1_id-1];
            parser::Vec3f c1 = scene->vertex_data[scene->triangles[minI].indices.v2_id-1];
            parser::Vec3f e1 = sub(b1,a1);
            parser::Vec3f e2 = sub(c1,b1);
            Norm = normalize(crossP(e1,e2));
        }
        else if(n==3){//MESH
            parser::Vec3f a1 = scene->vertex_data[scene->meshes[minI].faces[faceI].v0_id-1];
            parser::Vec3f b1 = scene->vertex_data[scene->meshes[minI].faces[faceI].v1_id-1];
            parser::Vec3f c1 = scene->vertex_data[scene->meshes[minI].faces[faceI].v2_id-1];
            parser::Vec3f e1 = sub(b1,a1);
            parser::Vec3f e2 = sub(c1,b1);
            Norm = normalize(crossP(e1,e2));

        }

        for (int j=0; j<noOfLights; j++){
            ray shadowRay;
            float s_minT = INFINITY;
            float s_minI = -1;
            float st;
            parser::Vec3f wi = sub(scene->point_lights[j].position, intersectionPoint);
            shadowRay.d = sub(scene->point_lights[j].position,intersectionPoint);
            shadowRay.d = normalize(shadowRay.d);

            parser::Vec3f epsilon;
            epsilon.x = shadowRay.d.x * scene->shadow_ray_epsilon;
            epsilon.y = shadowRay.d.y * scene->shadow_ray_epsilon;
            epsilon.z = shadowRay.d.z * scene->shadow_ray_epsilon;

            shadowRay.o = add(intersectionPoint, epsilon);

            
             for(int k=0;k<scene->spheres.size();k++){
                    st = intersectSphere(shadowRay,scene->spheres[k],scene);
                    if (st<s_minT && st>scene->shadow_ray_epsilon){
                        s_minT=st;
                        s_minI=k;
                    }   
                }
                for(int k=0;k<scene->triangles.size();k++){
                    st = intersectTriangle(shadowRay,scene->triangles[k].indices,scene);
                    if (st<s_minT && st>scene->shadow_ray_epsilon){
                        s_minT=st;
                        s_minI=k;
                
                    }   
                }
                for(int k=0;k<scene->meshes.size();k++){
                for(int l=0;l<scene->meshes[k].faces.size();l++){
                    st = intersectTriangle(shadowRay,scene->meshes[k].faces[l],scene);
                    if (st<s_minT && st>scene->shadow_ray_epsilon){//care about this
                        s_minT=st;
                        s_minI=k;
                        

                    }   
                }
                }

            if(s_minI==-1 || s_minT >= sqrt(wi.x*wi.x+wi.y*wi.y+wi.z*wi.z)){ //no shadow
                
                parser::Vec3f diffuse;
                parser::Vec3f irradiance;
                parser::Vec3f specular;
                //find irradence
                float r_2 = dotP(sub(scene->point_lights[j].position, intersectionPoint),sub(scene->point_lights[j].position, intersectionPoint));
                if(r_2 != 0){
                    irradiance.x = scene->point_lights[j].intensity.x/r_2;
	                irradiance.y = scene->point_lights[j].intensity.y/r_2;
	                irradiance.z =scene->point_lights[j].intensity.z/r_2;
                }
                else irradiance.x=irradiance.y=irradiance.z=0;

                //find diffuse
                parser::Vec3f lx = sub(scene->point_lights[j].position, intersectionPoint);
	            lx = normalize(lx);
                float lxn = dotP(lx, Norm);
                if(lxn<0) lxn=0;
                diffuse.x = scene->materials[c_materialId-1].diffuse.x * lxn * irradiance.x;
                diffuse.y = scene->materials[c_materialId-1].diffuse.y * lxn * irradiance.y;
                diffuse.z = scene->materials[c_materialId-1].diffuse.z * lxn * irradiance.z;

                //find specular
                parser::Vec3f wi = sub(scene->point_lights[j].position, intersectionPoint);
	            wi = normalize(wi);

	            parser::Vec3f h = sub(wi, r.d);
	            h = normalize(h);

                float hwi = dotP(Norm,h);
                if(hwi<0) hwi = 0;

                specular.x = scene->materials[c_materialId-1].specular.x * pow(hwi,scene->materials[c_materialId-1].phong_exponent) * irradiance.x;
                specular.y = scene->materials[c_materialId-1].specular.y * pow(hwi,scene->materials[c_materialId-1].phong_exponent) * irradiance.y;
                specular.z = scene->materials[c_materialId-1].specular.z * pow(hwi,scene->materials[c_materialId-1].phong_exponent) * irradiance.z;

                c.x += diffuse.x + specular.x;
                c.y += diffuse.y + specular.y; 
                c.z += diffuse.z + specular.z;

                //reflection
                if(recursionDepth>0){
                    if(scene->materials[c_materialId-1].mirror.x > 0 || scene->materials[c_materialId-1].mirror.y > 0 || scene->materials[c_materialId-1].mirror.z > 0){
                        parser::Vec3f reflection;
                        float coso = -2 * dotP(r.d, Norm);
                        parser::Vec3f reflection_d;
                        reflection_d.x = coso * Norm.x + r.d.x;
                        reflection_d.y = coso * Norm.y + r.d.y;
                        reflection_d.z = coso * Norm.z + r.d.z;

                        reflection_d = normalize(reflection_d);


                        ray reflectionRay = { add(intersectionPoint, epsilon),reflection_d};
                        reflection = computeColor(reflectionRay, scene, recursionDepth-1);

                        c.x += reflection.x * scene->materials[c_materialId-1].mirror.x;
      		            c.y += reflection.y * scene->materials[c_materialId-1].mirror.y;
      		            c.z += reflection.z * scene->materials[c_materialId-1].mirror.z;
                    }
                }
            }
            
        
        }
                    
    }
    else{
        c.x = scene->background_color.x;
        c.y = scene->background_color.y;
        c.z = scene->background_color.z;
        
    }
    return c;

}  
    








int main(int argc, char* argv[])
{
    parser::Scene sceneMain;
    parser::Scene *scene = &sceneMain;

    scene->loadFromXml(argv[1]);

    int numberOfCameras = scene->cameras.size();

    for(int cameraNo = 0; cameraNo < numberOfCameras; cameraNo++){
        parser::Camera camera = scene->cameras[cameraNo];
        int width = scene->cameras[cameraNo].image_width;
        int height = scene->cameras[cameraNo].image_height;
        float minT=INT32_MAX;
        unsigned char* image = new unsigned char [width * height * 3];
        int i = 0;
        float progress = 0;
        int progress2 = 0;

        for (int y = 0; y < height; ++y){
            progress=(((float)y)/height)*100;
            /*if(progress2 != (int) progress){
                progress2 = (int) progress;
                std::cout << progress2 << "%" << std::endl;
            }*/
            
            for (int x = 0; x < width; ++x){

                ray myray = generateRay(camera, x, y);

       
                parser::Vec3f rayColor;
            
                rayColor=computeColor(myray,scene,scene->max_recursion_depth);

           
                if(rayColor.x > 255) image[i] = 255;
              	else image[i] = round(rayColor.x);
                i++;
            	if(rayColor.y > 255) image[i] = 255;
             	else image[i] = round(rayColor.y);
                i++;
            	if(rayColor.z > 255) image[i] = 255;
              	else image[i] = round(rayColor.z);
                i++;
            }
        } 
        write_ppm(camera.image_name.c_str(), image, width, height);  

    }
    
}
