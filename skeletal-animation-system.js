var blendDualQuaternions = require('./blend-dual-quaternions.js')

module.exports = {
  interpolateJoints: interpolateJoints
}

// TODO: Finishing adding comments
// TODO: Benchmark performance and optimize
function interpolateJoints (opts) {
  // Get the amount of time that the current animation has been running.
  // We use this when interpolating the current animation. Depending on
  // how long the animation has been running we'll sample from different
  // keyframe times
  var currentAnimElapsedTime = opts.currentTime - opts.currentAnimation.startTime

  // Sort all of our animations keyframe times numerically so that
  // they're next to each other when we're sampling them.
  // ex: {1: [...], '6.5': [...], 2: [...]} becomes [1, 2, 6.5]
  // All keyframe times are in seconds
  var currentKeyframeTimes = Object.keys(opts.currentAnimation.keyframes)
  .sort(function (a, b) {
    // NOTE: This breaks if you have the same keyframe twice. But you
    // really shouldn't have the same keyframe twice. In the future
    // we might have a separate package for linting your model
    return Number(a) > Number(b) ? 1 : -1
  })
  var previousKeyframeTimes
  if (opts.previousAnimation) {
    previousKeyframeTimes = Object.keys(opts.previousAnimation.keyframes)
    .sort(function (a, b) {
      return Number(a) > Number(b) ? 1 : -1
    })
  }

  // Get the current animation's time relative to the first possible time.
  // For example, if our keyframe times are [1, 2, 6.5] and the current animation's
  // elapsed time is `3`, then our time relative to our first time is `1 + 3` or `4`
  var timeRelativeToFirst = Number(currentKeyframeTimes[0]) + Number(currentAnimElapsedTime)
  // Our duration is the number of seconds from the first keyframe time to the last
  // in our current animation. So for a current animation of [1, 2, 6.5] our duration is 4.5
  var duration = currentKeyframeTimes[currentKeyframeTimes.length - 1] - currentKeyframeTimes[0]
  if (currentAnimElapsedTime > duration) {
    // If we are NOT LOOPING then we set our upper bound of elapsed time to the duration of the animation
    if (opts.currentAnimation.noLoop) {
      currentAnimElapsedTime = Math.min(currentAnimElapsedTime, duration)
    } else {
      // If we ARE LOOPING then we use the modulus of the animation duration to
      // always re-start from the beginning
      currentAnimElapsedTime = currentAnimElapsedTime % duration
    }
    timeRelativeToFirst = currentAnimElapsedTime + Number(currentKeyframeTimes[0])
  }

  var currentAnimLowerKeyframe
  var currentAnimUpperKeyframe
  // Get the surrounding keyframes for our current animation
  currentKeyframeTimes.forEach(function (keyframeTime) {
    if (currentAnimLowerKeyframe && currentAnimUpperKeyframe) { return }
    if (timeRelativeToFirst > keyframeTime) {
      currentAnimLowerKeyframe = keyframeTime
    } else if (timeRelativeToFirst < keyframeTime) {
      currentAnimUpperKeyframe = keyframeTime
    } else if (timeRelativeToFirst === Number(keyframeTime)) {
      // TODO: Perform fewer calculations in places that we already know
      // that the keyframe time doesn't need to be blended against an upper
      // and lower keyframe. For now we don't handle this special case
      currentAnimLowerKeyframe = currentAnimUpperKeyframe = keyframeTime
    }
  })
  // Set the elapsed time relative to our current lower bound keyframe instead of our lowest out of all keyframes
  currentAnimElapsedTime = timeRelativeToFirst - currentAnimLowerKeyframe

  var previousAnimLowerKeyframe
  var previousAnimUpperKeyframe
  var prevAnimElapsedTime
  if (opts.previousAnimation) {
    var previousKeyframeData = require('./get-previous-animation-data.js')(opts, previousKeyframeTimes)
    previousAnimLowerKeyframe = previousKeyframeData.lower
    previousAnimUpperKeyframe = previousKeyframeData.upper
    prevAnimElapsedTime = previousKeyframeData.elapsedTime
  }

  // Calculate the interpolated joint matrices for our consumer's animation
  // TODO: acc is a bad variable name. Renaame it
  var interpolatedJoints = opts.jointNums.reduce(function (acc, jointName) {
    // If there is a previous animation
    var blend = (opts.blendFunction || defaultBlend)(opts.currentTime - opts.currentAnimation.startTime)

    if (opts.previousAnimation && blend < 1) {
      var previousAnimJointDualQuat
      var currentAnimJointDualQuat

      if (previousAnimLowerKeyframe === previousAnimUpperKeyframe) {
        // If our current frame happens to be one of our defined keyframes we use the existing frame
        previousAnimJointDualQuat = opts.previousAnimation.keyframes[previousAnimLowerKeyframe][jointName]
      } else {
        // Blend the dual quaternions for our previous animation that we are about to blend out
        previousAnimJointDualQuat = blendDualQuaternions(
          [],
          opts.previousAnimation.keyframes[previousAnimLowerKeyframe][jointName],
          opts.previousAnimation.keyframes[previousAnimUpperKeyframe][jointName],
          prevAnimElapsedTime / (previousAnimUpperKeyframe - previousAnimLowerKeyframe)
        )
      }

      if (currentAnimLowerKeyframe === currentAnimUpperKeyframe) {
        // If our current frame happens to be one of our defined keyframes we use the existing frame
        currentAnimJointDualQuat = opts.currentAnimation.keyframes[currentAnimLowerKeyframe][jointName]
      } else {
        currentAnimJointDualQuat = blendDualQuaternions(
          [],
          opts.currentAnimation.keyframes[currentAnimLowerKeyframe][jointName],
          opts.currentAnimation.keyframes[currentAnimUpperKeyframe][jointName],
          currentAnimElapsedTime / (currentAnimUpperKeyframe - currentAnimLowerKeyframe)
        )
      }

      acc[jointName] = blendDualQuaternions([], previousAnimJointDualQuat, currentAnimJointDualQuat, blend)
    } else {
      // If we are on an exact, pre-defined keyframe there is no need to blend
      if (currentAnimUpperKeyframe === currentAnimLowerKeyframe) {
        acc[jointName] = opts.currentAnimation.keyframes[currentAnimLowerKeyframe][jointName]
      } else {
        // Blend the two dual quaternions based on where we are in the current keyframe
        acc[jointName] = blendDualQuaternions(
          [],
          // The defined keyframe right below our current frame
          opts.currentAnimation.keyframes[currentAnimLowerKeyframe][jointName],
          // The defined keyframe right above our current frame
          opts.currentAnimation.keyframes[currentAnimUpperKeyframe][jointName],
          currentAnimElapsedTime / (currentAnimUpperKeyframe - currentAnimLowerKeyframe)
        )
      }
    }
    return acc
  }, {})

  // Calculate the keyframe number of our upper and lower keyframe
  // TODO: Handle this while we do other stuff so we don't need to loop through again
  //  this is a minor perf optimization that we can implement when we benchmark
  var currentAnimLowerKeyframeNumber
  var currentAnimUpperKeyframeNumber
  currentKeyframeTimes.forEach(function (keyTime, keyframeNumber) {
    currentAnimLowerKeyframeNumber = currentAnimLowerKeyframe === keyTime ? keyframeNumber : currentAnimLowerKeyframeNumber
    currentAnimUpperKeyframeNumber = currentAnimUpperKeyframe === keyTime ? keyframeNumber : currentAnimUpperKeyframeNumber
  })

  // Return the freshly interpolated dual quaternions for each of the joints that were passed in
  return {
    joints: interpolatedJoints,
    currentAnimationInfo: {
      lowerKeyframeNumber: currentAnimLowerKeyframeNumber,
      upperKeyframeNumber: currentAnimUpperKeyframeNumber
    }
  }
}

// Give then number of seconds elapsed between the previous animation
// and the current animation we return a blend factor between
// zero and one
function defaultBlend (dt) {
  // If zero time has elapsed we avoid dividing by 0
  if (!dt) { return 0 }
  // Blender linearly over 0.5s
  return 2 * dt
}
















/**

1. Cowboy.JSON importado (quemado en el codigo)
2. Main !
    canvas, gl
    playbackSlider: acelerar o ralenjtizar
    demoLocation se inserta en la web page
    cowboyJSON toda la informaci󮠤el personaje:
        articulaciones y posiciones por keyframe y coordenadas UV
        .keyframe matrices convertidas a cuaterniones duales
    cowboymodel.shader shader program cargado. Se le asigna la textura con 
        Script loadCollada
    renderLoop. Script raf-loop JS. Realiza la animacion frame por frame.
    anymationSystem. Script del skeletal-animation-system
    renderLoop function 
        crea las variables uniformes (luz ambientes y matrices MVP)
        interpolatedJoints. articulaciones interpoladas entre keyfame y keyframe
            18x8 floats. 
3.  getAtributesUniforms
    funciones de manejo de matrices retornar una mat3, crear una mat4, multiplica dos mat4 (4,5,6)
    mat perspective, rotateX, rotateY, rotateZ, translate 7,8,9,10,11
    manejo de quaternions: quat desde mat3, mult quat, escala - 12,13,14
    manejo de vectores: normaliza , escalar 15, 16,17
18. manejo de node.js, funcion inherits 
19  convertKeyframesToDualQuats!! convierte cada matriz de cada keyframe en quaterniones dobles ***
20  drawModel, el draw con el gl, data, donde se conectan los atributos y uniformes!
        texture
        conectaados los atributos de de normals, joints, weights y las variables uniforms como las matrices y la luz
        El modelo es dibujado a punta de triangulos
        luz
        MV y P Matrix
        dibujar Elements (triangles)
21  createDrawFunction. crea los datos necesarios para dibujar en Javascript
        allUniform: cada uniform crea un JS string
22  expandVertces, nada que ver ni interesante
23  initTexture
24  loadColladaDae, 
        shader tiene vertexPos, vertexNormal, jointIndex, joingWeight, vertexPosotinIndexBuffer y textura en dado caso
        createBuffer
25  generateFragmentShader genera el fragmentShader en GLSL
26  generateShader genera el shader con los strings de ambos shaders
27  generateVertexShader
28  convertMatrixToDualQuat !!!
        crea una matriz 3x3 a partir de una matriz 4x4
        4.12.13.14
        un quaternion correponde a la rotacion, otro a la traslacion
        
        
29  performance-now: define el tiempo actual 
30*  raf-loop: Clase Engine. Dedicada a movilizar la animacion de acuerdo a un tick
31  el famoso raf
32  right now
33  imoprt skeletal anumation system 
    
34*  blendDualQuaternions "interpolan" cuaterniones
        lerpDualQuaternions, lerpNegatedDialQuaternions
35*  getPreviousAnimiationData TODO debug a ver que eslo que sale
36*  interpolatedJoints!! Interpolar joints entre keyframes interpolando los cuaterniones
        defaultBlend: segundos entre la ultima animacion y ya
37  Clase EvenEmitter: clase que maneja responsabilidades tipo EventListener para ejecucion de Node
38  process calculo del tiempo actual en el browser
    
       

**/





















