/* ESP32: destinos absolutos por tabla experimental media.
 * Drivers configurados FISICAMENTE a 800 pulsos/vuelta.
 * Limite absoluto +/-750 pulsos desde el cero manual.
 * Angulos +/-30 grados; negativos por simetria NO medida.
 * Movimientos combinados secuenciales: motor 1 y luego motor 2.
 * Posicion estimada por pulsos, sin encoder ni homing automatico.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <stdlib.h>
#include <errno.h>

WebServer server(80);

const char* nombreRed = "HELIOSTATO_MANUAL";
const char* claveRed = "manual123";

const int dirPin[2] = {32, 26};
const int stepPin[2] = {33, 25};
const int enablePin[2] = {27, 23};

// Cambia a true si el sentido de un eje esta invertido.
// Si cambias esto, debes volver a calibrar.
const bool invertirDir[2] = {false, false};

// Intervalo entre pulsos completos.
constexpr uint32_t intervaloFijoUs = 100000; // 10 pulsos/s nominales.
constexpr long recorridoMedido = 750;
constexpr int pulsosPorVuelta = 800; // Configuracion fisica del driver, no del espejo.
// La tabla no compensa el acoplamiento mecanico entre ejes.
const bool permitirCombinados = true;

const uint32_t anchoPulsoUs = 20;
const uint32_t preparacionUs = 3000;
const uint32_t conexionMaxMs = 2500;



long posicion[2] = {0, 0};
long limiteMin[2] = {0, 0};
long limiteMax[2] = {0, 0};

bool tieneMin[2] = {false, false};
bool tieneMax[2] = {false, false};

bool origenEstablecido = false;
bool rangoConfirmado = false;
bool modoSeguro = true;

bool moviendo = false;
bool pulsoAlto = false;
bool detenerPendiente = false;

int ejeActivo = 0;
int sentidoActivo = 1;

long pasosTotales = 0;
long pasosEmitidos = 0;

uint32_t ultimoCambioUs = 0;
uint32_t esperaUs = 0;
uint32_t ultimoContactoMs = 0;

bool jogActivo = false;
uint32_t contactoJogMs = 0;
String tokenJog;
constexpr uint32_t vigenciaJogMs = 450;
bool segundoPendiente = false;
long segundoDestino = 0;
bool respuestaInterna = false;

// Suma de ambas mediciones: mantiene los medios pasos sin redondeo previo.
constexpr uint16_t pasosDobles[31] = {
  0,50,97,141,194,241,292,342,392,434,483,536,582,628,673,
  722,769,816,858,906,941,989,1031,1079,1122,1164,1207,1249,
  1289,1332,1373
};

String mensaje =
  "Coloca ambos ejes en su referencia y establece el origen.";

// Declaraciones explicitas de las funciones.
void responder(int codigo, const String& texto);
void activarModoSeguro(const String& motivo);
bool comprobarDetenido();
int leerEje();
bool iniciarMovimiento(int eje, long destino);

void actualizarMovimiento();
void enviarEstado();

bool leerGrados(const char* nombre, double& valor);
long gradosAPasos(double grados);
bool iniciarCombinado(long destino1, long destino2);
void continuarCombinado();

const char pagina[] PROGMEM = R"HTML(
<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Heliostato: manual y preparacion solar</title>

<style>
* { box-sizing: border-box; }
[hidden] { display: none !important; }
body {
  font-family: Arial, sans-serif;
  background: #101011;
  color: #ededee;
  max-width: 960px;
  margin: auto;
  padding: 18px;
  text-align: center;
}

section {
  background: #222225;
  border: 1px solid #595044;
  border-left: 3px solid #d4ad36;
  border-radius: 12px;
  padding: 15px;
  margin: 16px 0;
}

button, select, input {
  font-size: 17px;
  padding: 13px;
  margin: 5px;
  border: 1px solid #a48a39;
  background: #343437;
  color: #f4f4f4;
  border-radius: 7px;
}

button {
  cursor: pointer;
}

button:disabled {
  opacity: 0.4;
  cursor: default;
}

.rojo {
  background: #721e27;
  color: white;
}

.verde {
  background: #52202a;
}
button:hover:not(:disabled) { border-color: #f5cd56; background: #65232c; }
button:focus-visible, input:focus-visible, select:focus-visible { outline: 2px solid #f5cd56; outline-offset: 3px; }
header { border-bottom: 2px solid #d4ad36; padding: 12px 0 22px; margin-bottom: 20px; }
h1 { font-size: clamp(25px, 5vw, 36px); margin: 8px 0; }
h2 { font-size: 21px; }
p { line-height: 1.6; }
.etiqueta { color: #e1bf57; text-transform: uppercase; letter-spacing: 2px; font-size: 12px; }
.paso { background: #411b22; padding: 14px; border: 1px solid #bd9c39; border-radius: 10px; }
#ejes { display: grid; grid-template-columns: repeat(2,minmax(0,1fr)); gap: 12px; }
#ejes section { margin: 0 0 16px; }
input { width: 140px; max-width: 100%; }
label { display: inline-block; margin: 6px; }
[data-mover] { font-size: 28px; min-width: 75px; background: #4b2027; }
@media(max-width: 600px) { #ejes { grid-template-columns: 1fr; } body { padding: 12px; } button { max-width: 100%; } }

.dato {
  font-size: 20px;
}

#error {
  color: #ffd17d;
  min-height: 25px;
}

#estado {
  min-height: 45px;
}

body { background:#edf2f0;color:#34423e;max-width:640px;padding:20px; }
header {border:0;margin:0;padding:16px 0;} h1 {font-size:30px;font-weight:500;}
.etiqueta {display:none;} h2 {font-size:20px;font-weight:500;}
section {background:#f9fbfa;border:1px solid #d7e0dc;border-radius:24px;padding:18px;margin:16px 0;}
button,select,input {background:#eff5f2;color:#34423e;border:1px solid #cedad4;border-radius:18px;}
button:hover:not(:disabled) {background:#dcece3;border-color:#a9beb1;}
.rojo {background:#f1d9d7;color:#713e37;} .verde {background:#d8e9df;color:#344b3e;}
.paso {background:transparent;border:0;color:#62736b;padding:4px;}
#ejes {display:block;} .cruceta {display:grid;grid-template-columns:repeat(3,90px);grid-template-rows:repeat(3,80px);gap:10px;justify-content:center;}
.cruceta button {touch-action:none;user-select:none;-webkit-user-select:none;font-size:44px;margin:0;min-width:0;background:#eff5f2;}
.arriba {grid-area:1/2;} .izquierda {grid-area:2/1;} .centro {grid-area:2/2;align-self:center;font-size:30px;} .derecha {grid-area:2/3;} .abajo {grid-area:3/2;}
#error {color:#85583c;} #estado,#ayudaControl {font-size:15px;} .dato {font-size:15px;}
</style>
</head>

<body>

<header>
<div class="etiqueta">Heliostato · Control de dos motores</div>
<h1 id="tituloPantalla">Heliostato · v4</h1>
<p id="subtituloPantalla">Establece el origen manual para acceder al panel.</p>
</header>

<button class="rojo" onclick="orden('/seguro', true)">
  Detener
</button>

<button id="habilitar"
        class="verde"
        data-orden
        disabled
        onclick="orden('/habilitar')">
  Habilitar
</button>

<p id="estado">Conectando...</p>
<p id="error" role="status" aria-live="polite"></p>
<p id="ayudaControl">Conectando con el controlador...</p>

<div id="calibracion">
<p class="paso">Nivela el espejo con el equipo apagado. Enciende y registra el cero.</p>
<section>
  <h2>Referencia</h2>

  <p>
    Coloca el espejo nivelado en la referencia fisica de ambos motores.
    Este boton asigna cero a ambos ejes; no mueve los motores.
  </p>

  <button id="origen"
          data-orden
          disabled
          onclick="referenciar()">
    Establecer origen de ambos ejes
  </button>

  <p>
    Se aplicaran limites de -750 a +750 pasos.
    Los drivers quedaran habilitados para conservar torque.
  </p>
</section>

<div id="controlesCalibracion">
<div id="controles">
<section>
  <h2>Posicion absoluta del espejo</h2>
  <label>Motor 1 (grados)<input id="grado1" type="number" min="-30" max="30" step="0.1" value="0"></label>
  <label>Motor 2 (grados)<input id="grado2" type="number" min="-30" max="30" step="0.1" value="0"></label>
  <p><button id="posicionar" data-orden disabled onclick="enviarPosicionAngular()">Ir a la posicion</button></p>
  <p>Destino desde el origen: primero motor 1 y luego motor 2.
  Conversion media experimental. Angulos negativos estimados por simetria.
  La posicion real no se mide automaticamente.</p>
</section>
<h2>Movimiento manual</h2>


<section><p>Velocidad fija: 10 pasos/s</p>
<button id="homeAmbos" data-orden disabled onclick="orden('/home-ambos')">Origen</button></section>
<div id="ejes"></div>
</div>
</div>


</div>
<main id="panel">



<div id="controlesPanel"></div>

</main>
<script>
let pulsacion = null;
let liberando = false;
let ocupado = false;
let conectado = false;
let estado = null;
let revision = 0;

const nombres = ['Motor 1', 'Motor 2'];

document.getElementById('ejes').innerHTML = `
<div class="cruceta">
<button class="arriba" data-mover disabled onpointerdown="comenzarJog(event,0,1)" onpointerup="soltarJog()" onpointercancel="soltarJog()" onlostpointercapture="soltarJog()" aria-label="Motor 1 positivo">↑</button>
<button class="izquierda" data-mover disabled onpointerdown="comenzarJog(event,1,-1)" onpointerup="soltarJog()" onpointercancel="soltarJog()" onlostpointercapture="soltarJog()" aria-label="Motor 2 negativo">←</button>
<span class="centro">·</span>
<button class="derecha" data-mover disabled onpointerdown="comenzarJog(event,1,1)" onpointerup="soltarJog()" onpointercancel="soltarJog()" onlostpointercapture="soltarJog()" aria-label="Motor 2 positivo">→</button>
<button class="abajo" data-mover disabled onpointerdown="comenzarJog(event,0,-1)" onpointerup="soltarJog()" onpointercancel="soltarJog()" onlostpointercapture="soltarJog()" aria-label="Motor 1 negativo">↓</button>
</div><p>Mantén una flecha presionada para mover. Suelta para detener.</p>
<p id="dato0">Motor 1: pendiente</p><p id="dato1">Motor 2: pendiente</p>`;

function pintarBotones() {
  document.querySelectorAll('[data-orden]').forEach(b => {
    b.disabled = true;
  });

  if (!conectado || !estado || ocupado) {
    return;
  }

  const e = estado;
  const detenido = !e.moviendo;
  document.getElementById('ayudaControl').textContent = !e.origen
    ? 'Para mover: primero establece el origen con el espejo horizontal.'
    : e.seguro ? 'Origen registrado. Pulsa Habilitar movimiento.'
    : e.moviendo ? 'Movimiento en curso. Puedes detenerlo con Detener.'
    : 'Listo: usa las flechas o introduce ambos angulos y pulsa Ir a la posicion.';
  document.getElementById('tituloPantalla').textContent = 'Heliostato · v4';
  document.getElementById('subtituloPantalla').textContent = 'Movimiento manual y destinos absolutos por angulos.';


  document.getElementById('origen').disabled = !detenido;

  document.getElementById('homeAmbos').disabled = !(e.origen && !e.seguro && detenido);

  document.getElementById('habilitar').disabled =
    !(e.origen && e.seguro && detenido);

  document.querySelectorAll('[data-mover]').forEach(b => {
    b.disabled = !(e.origen && !e.seguro && (detenido || pulsacion !== null));
  });

  document.querySelectorAll('[data-limite]').forEach(b => {
    b.hidden = e.calibrado;
    b.disabled = !(e.origen && !e.calibrado && detenido);
  });

  document.querySelectorAll('[data-home]').forEach(b => {
    b.hidden = false;
    b.disabled = !(e.origen && !e.seguro && detenido);
  });

  document.getElementById('posicionar').disabled = !(e.origen && !e.seguro && detenido);
}

async function peticion(url, opciones = {}) {
  const controlador = new AbortController();

  const temporizador = setTimeout(() => {
    controlador.abort();
  }, 1800);

  try {
    const respuesta = await fetch(url, {
      ...opciones,
      cache: 'no-store',
      signal: controlador.signal
    });
    // Mantener el timeout hasta recibir tambien el cuerpo.
    const cuerpo = await respuesta.text();
    return {ok: respuesta.ok, text: async () => cuerpo, json: async () => JSON.parse(cuerpo)};
  } finally {
    clearTimeout(temporizador);
  }
}

async function consultarEstado() {
  const versionConsulta = revision;

  try {
    const respuesta = await peticion('/estado');

    if (!respuesta.ok) {
      throw new Error('Sin respuesta');
    }

    const e = await respuesta.json();

    if (versionConsulta !== revision || ocupado) {
      return;
    }

    estado = e;
    conectado = true;

    let texto = e.seguro
      ? 'MODO SEGURO. '
      : e.moviendo
        ? 'MOVIENDO. '
        : 'HABILITADO. ';

    texto += e.calibrado
      ? 'Rango confirmado. '
      : e.origen
        ? 'Calibrando. '
        : 'Sin origen. ';

    texto += e.mensaje;


    document.getElementById('estado').textContent = texto;

    e.ejes.forEach((x, eje) => {
      const minimo = x.min === null ? 'pendiente' : x.min;
      const maximo = x.max === null ? 'pendiente' : x.max;

      document.getElementById('dato' + eje).textContent =
        `Posicion: ${x.pos} | Minimo: ${minimo} | Maximo: ${maximo}`;
    });

  } catch (error) {
    conectado = false;

    document.getElementById('estado').textContent =
      'Sin conexion. Reconecta y verifica el estado.';

  } finally {
    pintarBotones();
    setTimeout(consultarEstado, 250);
  }
}

async function orden(url, parada = false) {
  if (parada) soltarJog();
  if ((pulsacion || liberando) && !parada) return;
  if ((ocupado || !conectado || !estado) && !parada) {
    return;
  }

  revision++;
  ocupado = true;
  const revisionOrden = revision;
  pintarBotones();

  try {
    const respuesta = await peticion(url, {
      method: 'POST'
    });

    document.getElementById('error').textContent =
      await respuesta.text();

  } catch (error) {
    conectado = false;

    document.getElementById('error').textContent =
      'Orden sin confirmar. Verifica el estado antes de repetirla.';

  } finally {
    if (revisionOrden === revision) {
      ocupado = false;
      estado = null;
      pintarBotones();
    }
  }
}

function enviarPosicionAngular() {
  const entradas = ['grado1', 'grado2'].map(id => document.getElementById(id));
  if (entradas.some(x => x.value.trim() === '' || !x.checkValidity() || !Number.isFinite(Number(x.value)))) {
    document.getElementById('error').textContent = 'Introduce ambos angulos entre -30 y 30 grados.';
    return;
  }
  const parametros = new URLSearchParams({g1: entradas[0].value, g2: entradas[1].value});
  orden('/posicionar?' + parametros);
}

async function comenzarJog(event, eje, sentido) {
  if (event.button !== 0 || pulsacion || liberando || ocupado || !estado || estado.moviendo || estado.seguro || !estado.origen) return;
  event.preventDefault();
  const p = {token: String(Date.now()) + '-' + Math.random().toString(16).slice(2), activo:true};
  pulsacion = p;
  event.currentTarget.setPointerCapture(event.pointerId);
  try {
    let r = await peticion('/jog?eje='+eje+'&sentido='+sentido+'&token='+encodeURIComponent(p.token), {method:'POST'});
    if (!r.ok) throw Error(await r.text());
    while (p.activo) {
      await new Promise(resolve => setTimeout(resolve, 120));
      if (!p.activo) break;
      r = await peticion('/jog-vivo?token='+encodeURIComponent(p.token), {method:'POST'});
      if (!r.ok) throw Error(await r.text());
    }
  } catch (error) {
    document.getElementById('error').textContent = error.message;
  } finally {
    p.activo=false; liberando=true;
    try { await peticion('/jog-fin?token='+encodeURIComponent(p.token), {method:'POST'}); }
    catch (_) { document.getElementById('error').textContent='Sin confirmacion de parada. El control caduca automaticamente.'; }
    if (pulsacion === p) pulsacion=null;
    liberando=false;
  }
}
function soltarJog() {
  if (!pulsacion) return;
  pulsacion.activo=false;
  // Envio inmediato; el finally repite la parada despues de cualquier inicio en vuelo.
  peticion('/jog-fin?token='+encodeURIComponent(pulsacion.token), {method:'POST'}).catch(()=>{});
}
window.addEventListener('pointerup',soltarJog);
window.addEventListener('blur',soltarJog);
document.addEventListener('visibilitychange',()=>{if(document.hidden) soltarJog();});

function referenciar() {
  const confirmado = confirm(
    'Confirma que ambos ejes estan en su referencia fisica. ' +
    'Se asignara cero a las posiciones actuales de ambos motores.'
  );

  if (confirmado) {
    orden('/origen');
  }
}

consultarEstado();
</script>

</body>
</html>
)HTML";

// ----------------------------------------------------
// Respuesta HTTP
// ----------------------------------------------------

void responder(int codigo, const String& texto) {
  if (respuestaInterna) return;
  server.send(
    codigo,
    "text/plain; charset=utf-8",
    texto
  );
}

// ----------------------------------------------------
// Modo seguro
// ----------------------------------------------------

void activarModoSeguro(const String& motivo) {
  jogActivo = false;
  tokenJog = "";
  segundoPendiente = false;
  modoSeguro = true;
  detenerPendiente = true;
  mensaje = motivo;

  // Si hay un pulso HIGH, se completa antes de detenerse.
  if (!pulsoAlto) {
    moviendo = false;
    detenerPendiente = false;
  }

  // No deshabilitar los drivers:
  // conservar torque si ya estaban habilitados.
}

// ----------------------------------------------------
// Validaciones
// ----------------------------------------------------

bool comprobarDetenido() {
  if (moviendo || segundoPendiente) {
    responder(409, "Espera a que termine el movimiento.");
    return false;
  }

  return true;
}

int leerEje() {
  String valor = server.arg("eje");

  if (valor == "0") {
    return 0;
  }

  if (valor == "1") {
    return 1;
  }

  responder(400, "Eje invalido.");
  return -1;
}

// ----------------------------------------------------
// Inicio de movimiento hacia una posicion
// ----------------------------------------------------

bool iniciarMovimiento(int eje, long destino) {
  if (!comprobarDetenido()) {
    return false;
  }

  if (!origenEstablecido || modoSeguro) {
    responder(
      403,
      "Establece el origen y habilita el movimiento."
    );
    return false;
  }

  if (destino < -recorridoMedido || destino > recorridoMedido) {
    responder(400, "El destino supera el limite por software de +/-750 pulsos.");
    return false;
  }

  if (rangoConfirmado) {
    if (destino < limiteMin[eje] ||
        destino > limiteMax[eje]) {
      responder(
        400,
        "El movimiento completo excede el rango permitido."
      );
      return false;
    }
  }

  if (!permitirCombinados && destino != posicion[eje] && posicion[1 - eje] != 0) {
    responder(409, "Primero devuelve el otro motor a cero: recorrido combinado pendiente de verificar.");
    return false;
  }

  long diferencia = destino - posicion[eje];

  if (diferencia == 0) {
    responder(200, "Ya esta en la posicion solicitada.");
    return false;
  }

  ejeActivo = eje;
  sentidoActivo = diferencia > 0 ? 1 : -1;

  pasosTotales = diferencia > 0
    ? diferencia
    : -diferencia;

  pasosEmitidos = 0;
  pulsoAlto = false;
  detenerPendiente = false;

  digitalWrite(stepPin[eje], LOW);

  bool nivelDireccion =
    ((sentidoActivo > 0) != invertirDir[eje]);

  digitalWrite(
    dirPin[eje],
    nivelDireccion ? HIGH : LOW
  );

  digitalWrite(enablePin[eje], LOW);

  ultimoCambioUs = micros();
  esperaUs = preparacionUs;

  ultimoContactoMs = millis();
  moviendo = true;

  mensaje = "Movimiento en curso.";

  responder(200, "Movimiento iniciado.");
  return true;
}

// ----------------------------------------------------
// Rampa de aceleracion y frenado
// ----------------------------------------------------

bool leerGrados(const char* nombre, double& valor) {
  if (!server.hasArg(nombre)) return false;
  String texto = server.arg(nombre);
  texto.trim();
  if (texto.length() == 0 || texto.length() > 24) return false;
  char* fin = nullptr;
  errno = 0;
  valor = strtod(texto.c_str(), &fin);
  return fin != texto.c_str() && *fin == '\0' && errno != ERANGE &&
         isfinite(valor) && valor >= -30.0 && valor <= 30.0;
}

long gradosAPasos(double grados) {
  const double magnitud = fabs(grados);
  const int entero = int(magnitud);
  double pasos = pasosDobles[entero] / 2.0;
  if (entero < 30) {
    pasos += (pasosDobles[entero + 1] - pasosDobles[entero]) *
             (magnitud - entero) / 2.0;
  }
  return grados < 0 ? -lround(pasos) : lround(pasos);
}

bool iniciarCombinado(long destino1, long destino2) {
  if (!comprobarDetenido()) return false;
  if (!origenEstablecido || modoSeguro) {
    responder(403, "Establece el origen y habilita el movimiento.");
    return false;
  }
  const long destinos[2] = {destino1, destino2};
  // Validacion atomica de ambos destinos antes de emitir un pulso.
  for (int i = 0; i < 2; ++i) {
    if (destinos[i] < -recorridoMedido || destinos[i] > recorridoMedido ||
        (rangoConfirmado && (destinos[i] < limiteMin[i] || destinos[i] > limiteMax[i]))) {
      responder(400, "Un destino supera los limites de recorrido.");
      return false;
    }
  }
  if (destino1 == posicion[0]) return iniciarMovimiento(1, destino2);
  const bool iniciado = iniciarMovimiento(0, destino1);
  if (iniciado && destino2 != posicion[1]) {
    segundoDestino = destino2;
    segundoPendiente = true;
  }
  return iniciado;
}

void continuarCombinado() {
  if (!segundoPendiente || moviendo || modoSeguro) return;
  segundoPendiente = false;
  respuestaInterna = true;
  iniciarMovimiento(1, segundoDestino);
  respuestaInterna = false;
}

// ----------------------------------------------------
// Generacion de pulsos sin bloquear todo el movimiento
// ----------------------------------------------------

void actualizarMovimiento() {
  if (jogActivo && uint32_t(millis() - contactoJogMs) > vigenciaJogMs) {
    activarModoSeguro("Se solto el control o se perdio la comunicacion. Habilita para continuar.");
  }
  if (!moviendo) {
    return;
  }

  if (!modoSeguro &&
      (uint32_t)(millis() - ultimoContactoMs) >
        conexionMaxMs) {

    activarModoSeguro(
      "Parada por falta de comunicacion. Revisa la posicion."
    );
  }

  if (!moviendo) {
    return;
  }

  uint32_t ahora = micros();

  if ((uint32_t)(ahora - ultimoCambioUs) < esperaUs) {
    return;
  }

  ultimoCambioUs = ahora;

  if (pulsoAlto) {
    digitalWrite(stepPin[ejeActivo], LOW);
    pulsoAlto = false;

    if (detenerPendiente ||
        modoSeguro ||
        pasosEmitidos >= pasosTotales) {

      moviendo = false;
      detenerPendiente = false;

      if (!modoSeguro) {
        mensaje = "Movimiento terminado.";
      }

      return;
    }

    esperaUs =
      intervaloFijoUs - anchoPulsoUs;

  } else {
    if (modoSeguro) {
      moviendo = false;
      return;
    }

    long siguiente =
      posicion[ejeActivo] + sentidoActivo;

    if (siguiente < -recorridoMedido || siguiente > recorridoMedido ||
        (!permitirCombinados && posicion[1 - ejeActivo] != 0)) {
      activarModoSeguro("Movimiento fuera del recorrido verificado.");
      return;
    }

    if (rangoConfirmado) {
      if (siguiente < limiteMin[ejeActivo] ||
          siguiente > limiteMax[ejeActivo]) {

        activarModoSeguro(
          "Limite de software alcanzado."
        );
        return;
      }
    }

    digitalWrite(stepPin[ejeActivo], HIGH);

    posicion[ejeActivo] = siguiente;
    pasosEmitidos++;

    pulsoAlto = true;
    esperaUs = anchoPulsoUs;
  }
}

// ----------------------------------------------------
// Estado para la pagina web
// ----------------------------------------------------

void enviarEstado() {
  ultimoContactoMs = millis();

  String json;
  json.reserve(450);

  json = "{\"origen\":";
  json += origenEstablecido ? "true" : "false";

  json += ",\"calibrado\":";
  json += rangoConfirmado ? "true" : "false";

  json += ",\"seguro\":";
  json += modoSeguro ? "true" : "false";

  json += ",\"moviendo\":";
  json += (moviendo || segundoPendiente) ? "true" : "false";

  json += ",\"velocidad\":";
  json += String(1000000UL / intervaloFijoUs);
  json += ",\"mensaje\":\"";
  json += mensaje;

  json += "\",\"ejes\":[";

  for (int eje = 0; eje < 2; eje++) {
    if (eje > 0) {
      json += ",";
    }

    json += "{\"pos\":";
    json += String(posicion[eje]);

    json += ",\"min\":";

    if (tieneMin[eje]) {
      json += String(limiteMin[eje]);
    } else {
      json += "null";
    }

    json += ",\"max\":";

    if (tieneMax[eje]) {
      json += String(limiteMax[eje]);
    } else {
      json += "null";
    }

    json += "}";
  }

  json += "]}";

  server.sendHeader("Cache-Control", "no-store");

  server.send(
    200,
    "application/json; charset=utf-8",
    json
  );
}

// ----------------------------------------------------
// Configuracion
// ----------------------------------------------------

void setup() {
  Serial.begin(115200);


  for (int eje = 0; eje < 2; eje++) {
    digitalWrite(enablePin[eje], HIGH);
    digitalWrite(stepPin[eje], LOW);
    digitalWrite(dirPin[eje], LOW);

    pinMode(enablePin[eje], OUTPUT);
    pinMode(stepPin[eje], OUTPUT);
    pinMode(dirPin[eje], OUTPUT);
  }

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(nombreRed, claveRed)) {
    Serial.println("Error al iniciar el punto de acceso. Reinicia el equipo.");
    // Los drivers siguen deshabilitados y no existe un origen valido.
    return;
  }

  Serial.print("Abre en el navegador: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/home-ambos", HTTP_POST, []() {
    iniciarCombinado(0, 0);
  });

  server.on("/posicionar", HTTP_POST, []() {
    double g1, g2;
    if (!leerGrados("g1", g1) || !leerGrados("g2", g2)) {
      responder(400, "Introduce dos angulos numericos entre -30 y 30 grados.");
      return;
    }
    iniciarCombinado(gradosAPasos(g1), gradosAPasos(g2));
  });

  // Pagina principal
  server.on("/recalibrar", HTTP_POST, []() {
    activarModoSeguro("Calibracion abierta. Se conserva el cero contado; revisa la referencia.");
    rangoConfirmado = false;
    for (int eje = 0; eje < 2; eje++) {
      tieneMin[eje] = false;
      tieneMax[eje] = false;
      limiteMin[eje] = 0;
      limiteMax[eje] = 0;
    }
    responder(200, mensaje);
  });

  // Pagina principal
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store, max-age=0");
    server.send_P(
      200,
      "text/html; charset=utf-8",
      pagina
    );
  });

  // Consulta de estado
  server.on("/estado", HTTP_GET, enviarEstado);

  // Parada
  server.on("/seguro", HTTP_POST, []() {
    activarModoSeguro("Parada solicitada.");
    responder(200, "Modo seguro activado.");
  });

  // Establecer origen manual
  server.on("/origen", HTTP_POST, []() {
    if (!comprobarDetenido()) {
      return;
    }

    for (int eje = 0; eje < 2; eje++) {
      posicion[eje] = 0;
      limiteMin[eje] = -recorridoMedido;
      limiteMax[eje] = recorridoMedido;
      tieneMin[eje] = tieneMax[eje] = true;
      digitalWrite(enablePin[eje], LOW);
    }
    origenEstablecido = true;
    rangoConfirmado = true;
    modoSeguro = true;
    mensaje = "Origen establecido. Limites +/-750 pasos. Habilita el movimiento.";

    responder(200, mensaje);
  });

  // Habilitar movimientos
  server.on("/habilitar", HTTP_POST, []() {
    if (!comprobarDetenido()) {
      return;
    }

    if (!origenEstablecido) {
      responder(403, "Primero establece el origen.");
      return;
    }

    for (int eje = 0; eje < 2; eje++) {
      digitalWrite(enablePin[eje], LOW);
    }

    modoSeguro = false;
    ultimoContactoMs = millis();

    mensaje = "Listo para mover.";
    responder(200, mensaje);
  });

  server.on("/jog", HTTP_POST, []() {
    int eje = leerEje();
    String sentido = server.arg("sentido");
    String token = server.arg("token");
    if (eje < 0) return;
    if ((sentido != "1" && sentido != "-1") || token.length() < 1 || token.length() > 80) {
      responder(400, "Control manual invalido."); return;
    }
    const long destino = sentido == "1" ? recorridoMedido : -recorridoMedido;
    if (iniciarMovimiento(eje, destino)) {
      jogActivo = true; tokenJog = token; contactoJogMs = millis();
    }
  });
  server.on("/jog-vivo", HTTP_POST, []() {
    if (!jogActivo || modoSeguro || tokenJog != server.arg("token")) {
      responder(409, "Control manual terminado."); return;
    }
    contactoJogMs = millis(); ultimoContactoMs = millis();
    responder(200, "OK");
  });
  server.on("/jog-fin", HTTP_POST, []() {
    if (jogActivo && tokenJog == server.arg("token")) {
      jogActivo = false; tokenJog = "";
      detenerPendiente = true;
      if (!pulsoAlto) { moviendo = false; detenerPendiente = false; }
      mensaje = "Detenido. Listo para mover.";
    }
    responder(200, "Control soltado.");
  });

  // Guardar minimo o maximo


  // Activar limites de software

  // Volver al cero contado de un eje
  server.on("/home", HTTP_POST, []() {
    int eje = leerEje();
    if (eje < 0) {
      return;
    }
    iniciarMovimiento(eje, 0);
  });
  server.begin();
}
// ----------------------------------------------------
// Bucle principal
// ----------------------------------------------------
void loop() {
  if (jogActivo && uint32_t(millis() - contactoJogMs) > vigenciaJogMs) {
    activarModoSeguro("Control manual sin confirmacion. Pulsa Habilitar.");
  }
  // Detectar desconexion tambien estando quieto; exigir habilitar al volver.
  if (origenEstablecido && !modoSeguro &&
      (uint32_t)(millis() - ultimoContactoMs) > conexionMaxMs) {
    activarModoSeguro("Sin comunicacion: habilita de nuevo para continuar.");
  }
  actualizarMovimiento();
  continuarCombinado();
  // Completar el pulso HIGH antes de atender HTTP.
  if (!pulsoAlto) {
    server.handleClient();
  }
  actualizarMovimiento();
}
