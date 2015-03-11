var 
scene,
world,
camera,
controls,
cart,
shadowMapSettings = {
    width: 2048,
    height: 2048
},
cameraSettings = {
    far: 2000,
    near: 5,
},
//phyiscs
wheelGroundContactMaterial,
groundMaterial,
wheelMaterial,
chassisMaterial,
ground,
settings = {
    stepFrequency: 60,
    quatNormalizeSkip: 2,
    quatNormalizeFast: true,
    gx: 0,
    gy: 0,
    gz: 0,
    iterations: 3,
    tolerance: 0.0001,
    k: 1e6,
    d: 3,
    scene: 0,
    paused: false,
    rendermode: "solid",
    constraints: false,
    contacts: false,  // Contact points
    cm2contact: false, // center of mass to contact points
    normals: false, // contact normals
    axes: false, // "local" frame axes
    particleSize: 0.1,
    shadows: false,
    aabbs: false,
    profiling: false,
    maxSubSteps:3
},
lastCallTime,
updateBodies = [];

function init(){

    if (!Detector.webgl){
        Detector.addGetWebGLMessage();
    }

    // SCENE
    scene = new THREE.Scene();
    scene.fog = new THREE.Fog( 0x222222, 1000, cameraSettings.far );
    
    // Camera
    camera = new THREE.PerspectiveCamera(24, window.innerWidth / window.innerHeight, cameraSettings.near, cameraSettings.far);
    camera.up.set(0,0,1);
    camera.position.set(0,30,20);
    scene.add(camera);

    //controls
    controls = new THREE.OrbitControls( camera );
    controls.damping = 0.2;
    controls.noKeys = true;

    //cart
    cart = new THREE.Group();
    cart.add(camera);
    scene.add(cart);

    // LIGHTS
    ambient = new THREE.AmbientLight( 0x222222 );
    scene.add(ambient);

    light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(0, -1.75, 1.5);
    light.position.multiplyScalar(50);
    light.target.position.set(0, 0, 0);

    light.castShadow = true;

    light.shadowCameraNear = 10;
    light.shadowCameraFar = 300;//camera.far;
    // light.shadowCameraFov = 30;

    light.shadowMapBias = 0.0039;
    light.shadowMapDarkness = 0.5;
    light.shadowMapWidth = 2048;
    light.shadowMapHeight = 2048;

    var size = 40;
    light.shadowCameraLeft = -size;
    light.shadowCameraRight = size;
    light.shadowCameraTop = size;
    light.shadowCameraBottom = -size;

    cart.add(light);
    cart.add(light.target);

    // RENDERER
    renderer = new THREE.WebGLRenderer({ clearColor: 0x000000, clearAlpha: 1, antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    $container.append(renderer.domElement);

    renderer.setClearColor( scene.fog.color, 1 );
    renderer.autoClear = false;

    renderer.shadowMapEnabled = true;
    renderer.shadowMapSoft = true;

    // STATS
    stats = new Stats();
    stats.domElement.style.position = 'absolute';
    stats.domElement.style.top = '0px';
    $container.append(stats.domElement);

    // Material
    var materialColor = 0xdddddd;
    var solidMaterial = new THREE.MeshLambertMaterial( { color: materialColor } );
    //THREE.ColorUtils.adjustHSV( solidMaterial.color, 0, 0, 0.9 );
    var wireframeMaterial = new THREE.MeshLambertMaterial( { color: 0xffffff, wireframe:true } );
    this.currentMaterial = solidMaterial;
    var contactDotMaterial = new THREE.MeshLambertMaterial( { color: 0xff0000 } );
    var particleMaterial = this.particleMaterial = new THREE.MeshLambertMaterial( { color: 0xff0000 } );
};
function initPhysics(){
    //create physics world
    world = new CANNON.World();
    world.broadphase = new CANNON.NaiveBroadphase();

    world.broadphase = new CANNON.SAPBroadphase(world);
    world.gravity.set(0, 0, -9.81);
    world.defaultContactMaterial.friction = 0;

    groundMaterial = new CANNON.Material("groundMaterial");
    wheelMaterial = new CANNON.Material("wheelMaterial");
    wheelGroundContactMaterial = new CANNON.ContactMaterial(wheelMaterial, groundMaterial, {
        friction: 0.3,
        restitution: 0,
        contactEquationStiffness: 1000
    });


    chassisMaterial = new CANNON.Material("chassisMaterial");
    chassisContactMaterial = new CANNON.ContactMaterial(chassisMaterial, groundMaterial, {
        friction: 0.3,
        restitution: 0,
        contactEquationStiffness: 1000
    });

    // We must add the contact materials to the world
    world.addContactMaterial(wheelGroundContactMaterial);
    world.addContactMaterial(chassisContactMaterial);

    var chassisShape = new CANNON.Box(new CANNON.Vec3(2, 1,0.5));
    var chassisBody = new CANNON.Body({mass: 300});
    chassisBody.addShape(chassisShape);
    chassisBody.position.set(0, 0, 4);
    chassisBody.angularVelocity.set(0, 0, 0.5);
    scene.add(createMesh(chassisBody));
    updateBodies.push(chassisBody);

    var options = {
        radius: 0.8,
        directionLocal: new CANNON.Vec3(0, 0, -1),
        suspensionStiffness: 15,
        suspensionRestLength: 0.5,
        frictionSlip: 3,
        dampingRelaxation: 2.3,
        dampingCompression: 4.4,
        maxSuspensionForce: 100000,
        rollInfluence:  0.01,
        axleLocal: new CANNON.Vec3(0, 1, 0),
        chassisConnectionPointLocal: new CANNON.Vec3(1, 1, 0),
        maxSuspensionTravel: 0.5,
        customSlidingRotationalSpeed: -30,
        useCustomSlidingRotationalSpeed: true
    };

    // Create the vehicle
    vehicle = new CANNON.RaycastVehicle({
        chassisBody: chassisBody,
    });

    options.chassisConnectionPointLocal.set(1.3, 1.2, 0);
    vehicle.addWheel(options);

    options.chassisConnectionPointLocal.set(1.3, -1.2, 0);
    vehicle.addWheel(options);

    options.chassisConnectionPointLocal.set(-1.3, 1.2, 0);
    vehicle.addWheel(options);

    options.chassisConnectionPointLocal.set(-1.3, -1.2, 0);
    vehicle.addWheel(options);

    vehicle.addToWorld(world);

    var wheelBodies = [];
    for(var i=0; i<vehicle.wheelInfos.length; i++){
        var wheel = vehicle.wheelInfos[i];
        var cylinderShape = new CANNON.Cylinder(wheel.radius, wheel.radius, wheel.radius / 2, 20);
        var wheelBody = new CANNON.Body({ mass: 20 });
        var q = new CANNON.Quaternion();
        q.setFromAxisAngle(new CANNON.Vec3(1, 0, 0), Math.PI / 2);
        wheelBody.addShape(cylinderShape, new CANNON.Vec3(), q);
        wheelBodies.push(wheelBody);
        scene.add(createMesh(wheelBody));
        updateBodies.push(wheelBody);
    }

    // Update wheels
    world.addEventListener('postStep', function(){
        for (var i = 0; i < vehicle.wheelInfos.length; i++) {
            vehicle.updateWheelTransform(i);
            var t = vehicle.wheelInfos[i].worldTransform;
            wheelBodies[i].position.copy(t.position);
            wheelBodies[i].quaternion.copy(t.quaternion);
        }
    });

    //create height feild
    var matrix = [];
    var sizeX = 64,
        sizeY = 64;

    for (var i = 0; i < sizeX; i++) {
        matrix.push([]);
        for (var j = 0; j < sizeY; j++) {
            var height = Math.cos(i / sizeX * Math.PI * 5) * Math.cos(j/sizeY * Math.PI * 5) * 4 + 4;
            if(i===0 || i === sizeX-1 || j===0 || j === sizeY-1)
                height = 8;
            matrix[i].push(height);
        }
    }

    var hfShape = new CANNON.Heightfield(matrix, {
        elementSize: 200 / sizeX
    });
    ground = new CANNON.Body({ mass: 0 });
    ground.addShape(hfShape);
    ground.position.set(-sizeX * hfShape.elementSize / 2, -sizeY * hfShape.elementSize / 2, -1);
    world.add(ground);
    createMesh(ground);
    ground.mesh.position.copy(ground.position);
    scene.add(ground.mesh);
};

function update(time){
    requestAnimationFrame(update);

    cart.position.copy(vehicle.chassisBody.mesh.position);
    if(camera.position.z < 10){
        camera.position.z = 10;
    }
    controls.update();

    animate(time);
    updatePhysics(time);
    render(time);

    stats.update();
}
function updatePhysics(time){
    // Step world
    var timeStep = 1 / settings.stepFrequency;

    var now = Date.now() / 1000;

    if(!lastCallTime){
        // last call time not saved, cant guess elapsed time. Take a simple step.
        world.step(timeStep);
        lastCallTime = now;
        return;
    }

    var timeSinceLastCall = now - lastCallTime;

    world.step(timeStep, timeSinceLastCall, settings.maxSubSteps);

    lastCallTime = now;

    //update meshes
    for(var i=0; i<updateBodies.length; i++){
        var body = updateBodies[i]
        var mesh = body.mesh;
        if(mesh){
            mesh.position.copy(body.position);
            if(body.quaternion){
                mesh.quaternion.copy(body.quaternion);
            }
        }
    }
}
function animate(time){
}
function render(time){
    renderer.render(scene,camera);
}


$(document).ready(function() {
    $container = $('<div>').appendTo('body');
    init();
    initPhysics();
    update();

    //car controls
    document.onkeydown = handler;
    document.onkeyup = handler;

    var maxSteerVal = 0.5;
    var maxForce = 1000;
    function handler(event){
        var up = (event.type == 'keyup');

        if(!up && event.type !== 'keydown'){
            return;
        }

        vehicle.setBrake(0, 0);
        vehicle.setBrake(0, 1);
        vehicle.setBrake(0, 2);
        vehicle.setBrake(0, 3);

        switch(event.keyCode){
            case 38: // forward
                vehicle.applyEngineForce(up ? 0 : -maxForce, 2);
                vehicle.applyEngineForce(up ? 0 : -maxForce, 3);
                break;

            case 40: // backward
                vehicle.applyEngineForce(up ? 0 : maxForce, 2);
                vehicle.applyEngineForce(up ? 0 : maxForce, 3);
                break;

            case 39: // right
                vehicle.setSteeringValue(up ? 0 : -maxSteerVal, 0);
                vehicle.setSteeringValue(up ? 0 : -maxSteerVal, 1);
                break;

            case 37: // left
                vehicle.setSteeringValue(up ? 0 : maxSteerVal, 0);
                vehicle.setSteeringValue(up ? 0 : maxSteerVal, 1);
                break;

            case 32: 
                vehicle.chassisBody.position.z = 10;
                vehicle.chassisBody.velocity.set(0,0,0);
                vehicle.chassisBody.angularVelocity.set(0,0,0);
                vehicle.chassisBody.quaternion.y = 0;
                vehicle.chassisBody.quaternion.x = 0;
                vehicle.chassisBody.quaternion.z = 0;
                break;

            case 82: 
                vehicle.chassisBody.position.set(0,0,5);
                vehicle.chassisBody.velocity.set(0,0,0);
                vehicle.chassisBody.angularVelocity.set(0,0,0);
                vehicle.chassisBody.quaternion.y = 0;
                vehicle.chassisBody.quaternion.x = 0;
                vehicle.chassisBody.quaternion.z = 0;
                break;
        }
    }

    $(window).resize(function(){
        renderer.setSize(window.innerWidth, window.innerHeight);

        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
    })

    //pointer lock
    // var havePointerLock = 'pointerLockElement' in document || 'mozPointerLockElement' in document || 'webkitPointerLockElement' in document;
    // if (havePointerLock) {
    //     var element = document.body;

    //     var pointerlockchange = function ( event ) {
    //         if (document.pointerLockElement === element || document.mozPointerLockElement === element || document.webkitPointerLockElement === element ) {
    //             controls.enabled = true;

    //             $('#blocker').hide();
    //         } 
    //         else {
    //             controls.enabled = false;

    //             $('#blocker').css('display','-webkit-box');
    //             $('#blocker').css('display','-moz-box');
    //             $('#blocker').css('display','box');

    //             $('#instructions').show();
    //         }
    //     }

    //     var pointerlockerror = function ( event ) {
    //         $('#instructions').show();
    //     }

    //     $('#instructions').click(function(event){
    //         $('#instructions').hide();

    //         // Ask the browser to lock the pointer
    //         element.requestPointerLock = element.requestPointerLock || element.mozRequestPointerLock || element.webkitRequestPointerLock;

    //         if ( /Firefox/i.test( navigator.userAgent ) ) {
    //             var fullscreenchange = function ( event ) {

    //                 if ( document.fullscreenElement === element || document.mozFullscreenElement === element || document.mozFullScreenElement === element ) {

    //                     document.removeEventListener( 'fullscreenchange', fullscreenchange );
    //                     document.removeEventListener( 'mozfullscreenchange', fullscreenchange );

    //                     element.requestPointerLock();
    //                 }

    //             }

    //             document.addEventListener( 'fullscreenchange', fullscreenchange, false );
    //             document.addEventListener( 'mozfullscreenchange', fullscreenchange, false );

    //             element.requestFullscreen = element.requestFullscreen || element.mozRequestFullscreen || element.mozRequestFullScreen || element.webkitRequestFullscreen;

    //             element.requestFullscreen();
    //         } 
    //         else {
    //             element.requestPointerLock();
    //         }
    //     });

    //     // Hook pointer lock state change events
    //     document.addEventListener( 'pointerlockchange', pointerlockchange, false );
    //     document.addEventListener( 'mozpointerlockchange', pointerlockchange, false );
    //     document.addEventListener( 'webkitpointerlockchange', pointerlockchange, false );

    //     document.addEventListener( 'pointerlockerror', pointerlockerror, false );
    //     document.addEventListener( 'mozpointerlockerror', pointerlockerror, false );
    //     document.addEventListener( 'webkitpointerlockerror', pointerlockerror, false );
    // } 
    // else {
    //     $('#instructions').innerHTML = 'Your browser doesn\'t seem to support Pointer Lock API';
    // }
});

shape2mesh = function(body){
    var obj = new THREE.Object3D();

    for (var l = 0; l < body.shapes.length; l++) {
        var shape = body.shapes[l];

        var mesh;

        switch(shape.type){

        case CANNON.Shape.types.SPHERE:
            var sphere_geometry = new THREE.SphereGeometry( shape.radius, 8, 8);
            mesh = new THREE.Mesh( sphere_geometry, this.currentMaterial );
            break;

        case CANNON.Shape.types.PARTICLE:
            mesh = new THREE.Mesh( this.particleGeo, this.particleMaterial );
            var s = this.settings;
            mesh.scale.set(s.particleSize,s.particleSize,s.particleSize);
            break;

        case CANNON.Shape.types.PLANE:
            var geometry = new THREE.PlaneGeometry(10, 10, 4, 4);
            mesh = new THREE.Object3D();
            var submesh = new THREE.Object3D();
            var ground = new THREE.Mesh( geometry, this.currentMaterial );
            ground.scale.set(100, 100, 100);
            submesh.add(ground);

            ground.castShadow = true;
            ground.receiveShadow = true;

            mesh.add(submesh);
            break;

        case CANNON.Shape.types.BOX:
            var box_geometry = new THREE.BoxGeometry(  shape.halfExtents.x*2,
                                                        shape.halfExtents.y*2,
                                                        shape.halfExtents.z*2 );
            mesh = new THREE.Mesh( box_geometry, this.currentMaterial );
            break;

        case CANNON.Shape.types.CONVEXPOLYHEDRON:
            var geo = new THREE.Geometry();

            // Add vertices
            for (var i = 0; i < shape.vertices.length; i++) {
                var v = shape.vertices[i];
                geo.vertices.push(new THREE.Vector3(v.x, v.y, v.z));
            }

            for(var i=0; i < shape.faces.length; i++){
                var face = shape.faces[i];

                // add triangles
                var a = face[0];
                for (var j = 1; j < face.length - 1; j++) {
                    var b = face[j];
                    var c = face[j + 1];
                    geo.faces.push(new THREE.Face3(a, b, c));
                }
            }
            geo.computeBoundingSphere();
            geo.computeFaceNormals();
            mesh = new THREE.Mesh( geo, this.currentMaterial );
            break;

        case CANNON.Shape.types.HEIGHTFIELD:
            var geometry = new THREE.Geometry();

            var v0 = new CANNON.Vec3();
            var v1 = new CANNON.Vec3();
            var v2 = new CANNON.Vec3();
            for (var xi = 0; xi < shape.data.length - 1; xi++) {
                for (var yi = 0; yi < shape.data[xi].length - 1; yi++) {
                    for (var k = 0; k < 2; k++) {
                        shape.getConvexTrianglePillar(xi, yi, k===0);
                        v0.copy(shape.pillarConvex.vertices[0]);
                        v1.copy(shape.pillarConvex.vertices[1]);
                        v2.copy(shape.pillarConvex.vertices[2]);
                        v0.vadd(shape.pillarOffset, v0);
                        v1.vadd(shape.pillarOffset, v1);
                        v2.vadd(shape.pillarOffset, v2);
                        geometry.vertices.push(
                            new THREE.Vector3(v0.x, v0.y, v0.z),
                            new THREE.Vector3(v1.x, v1.y, v1.z),
                            new THREE.Vector3(v2.x, v2.y, v2.z)
                        );
                        var i = geometry.vertices.length - 3;
                        geometry.faces.push(new THREE.Face3(i, i+1, i+2));
                    }
                }
            }
            geometry.computeBoundingSphere();
            geometry.computeFaceNormals();
            mesh = new THREE.Mesh(geometry, this.currentMaterial);
            break;

        default:
            throw "Visual type not recognized: "+shape.type;
        }

        mesh.receiveShadow = true;
        mesh.castShadow = true;
        if(mesh.children){
            for(var i=0; i<mesh.children.length; i++){
                mesh.children[i].castShadow = true;
                mesh.children[i].receiveShadow = true;
                if(mesh.children[i]){
                    for(var j=0; j<mesh.children[i].length; j++){
                        mesh.children[i].children[j].castShadow = true;
                        mesh.children[i].children[j].receiveShadow = true;
                    }
                }
            }
        }

        var o = body.shapeOffsets[l];
        var q = body.shapeOrientations[l];
        mesh.position.set(o.x, o.y, o.z);
        mesh.quaternion.set(q.x, q.y, q.z, q.w);

        obj.add(mesh);
    }

    return obj;
};

createMesh = function(body){
    // What geometry should be used?
    var mesh;
    if(body instanceof CANNON.Body){
        mesh = shape2mesh(body);
    }
    if(mesh) {
        body.mesh = mesh;
        mesh.body = body;
    }

    return mesh;
};