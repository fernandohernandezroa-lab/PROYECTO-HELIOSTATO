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
const uint32_t conexionMaxMs = 15000; // 15 s sin comunicacion de la pestaña de control.



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
IPAddress clienteControl;
bool hayClienteControl = false;
String sesionControl;

bool jogActivo = false;
uint32_t contactoJogMs = 0;
String tokenJog;
constexpr uint32_t vigenciaJogMs = 450;
bool segundoPendiente = false;
long segundoDestino = 0;
bool respuestaInterna = false;
// Plan relativo al reloj monotono del ESP32; no necesita Internet.
constexpr int maxPuntosSolar = 37;
long planSolar[maxPuntosSolar][2];
int puntosSolar = 0, indiceSolar = 0;
bool solarActivo = false;
uint32_t inicioSolarMs = 0, demoraSolarMs = 0;
constexpr uint32_t periodoSolarMs = 300000;
void actualizarSolar();
void recibirPlanSolar();

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
*{box-sizing:border-box}[hidden]{display:none!important}
:root{font:15px system-ui,sans-serif;color:#263b31;background:#edf2ee}
body{margin:0}button,input,select{font:inherit;color:inherit;border:1px solid #ced9d1;border-radius:10px;background:#fff;min-height:42px;padding:9px 12px}
button{cursor:pointer}button:disabled{opacity:.4;cursor:default}button:hover:not(:disabled){background:#e4eee6}button:focus-visible,input:focus-visible,select:focus-visible,summary:focus-visible{outline:3px solid #6b9a7b;outline-offset:2px}
.app{height:100vh;height:100dvh;max-width:740px;margin:auto;padding:12px;display:flex;flex-direction:column;gap:8px;overflow:auto}
header{display:flex;align-items:center;justify-content:space-between;gap:8px}h1{font-size:21px;font-weight:600;margin:0}h2{font-size:16px;margin:0}p{margin:0;line-height:1.4}
.acciones{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}.rojo{background:#f3d8d4;color:#803b30}.verde{background:#dceade}.primario{background:#285c40;color:white}.primario:hover:not(:disabled){background:#346e4d}
#estado{font-size:13px;max-height:54px;overflow:auto}#error{font-size:13px;color:#943d2d}#error:empty{display:none}
section{background:white;border:1px solid #dae3dc;border-radius:16px;padding:12px}.manual{flex:1;min-height:242px;display:flex;flex-direction:column;justify-content:space-between;gap:6px}.cabecera{display:flex;align-items:center;justify-content:space-between;gap:6px}.cabecera button{min-height:36px;padding:5px 10px;font-size:13px}
.cruceta{display:grid;grid-template-columns:repeat(3,48px);grid-template-rows:repeat(3,44px);gap:4px;justify-content:center}.cruceta button{touch-action:none;user-select:none;-webkit-user-select:none;font-size:26px;padding:0;margin:0}.arriba{grid-area:1/2}.izquierda{grid-area:2/1}.centro{grid-area:2/2;align-self:center;text-align:center}.derecha{grid-area:2/3}.abajo{grid-area:3/2}
.manualContenido{display:grid;grid-template-columns:minmax(140px,1fr) minmax(0,1fr);gap:8px;align-items:center}.posiciones{display:flex;justify-content:center;gap:10px;font-size:11px;margin-top:4px}.angulos{display:grid;grid-template-columns:1fr 1fr;gap:6px;align-items:end}.angulos label{display:flex;flex-direction:column;font-size:12px;gap:3px}.angulos input{width:100%;min-width:0;height:42px;padding:6px}.angulos button{height:42px;grid-column:1/-1}
#solarPanel{display:flex;flex-direction:column;gap:7px}.intervalo{font-size:12px;color:#607568}.solarAcciones{display:grid;grid-template-columns:1fr 1fr;gap:6px}#resumenSolar{font-size:12px;max-height:52px;overflow:auto}footer{font-size:11px;text-align:center;color:#607568}
dialog{width:min(620px,calc(100% - 24px));max-height:90vh;max-height:90dvh;border:1px solid #cdd9d0;border-radius:18px;padding:16px;color:inherit}dialog::backdrop{background:#16251c88}.dialogCabecera{display:flex;justify-content:space-between;align-items:center;gap:8px;margin-bottom:12px}.dialogCabecera button{min-height:36px;padding:6px 12px}dialog p{font-size:12px;margin:8px 0}dialog label{display:flex;flex-direction:column;gap:4px;font-size:13px;margin:8px 0}dialog input,dialog select{width:100%;min-width:0}.campos{display:grid;grid-template-columns:1fr 1fr;gap:0 12px}.check{flex-direction:row!important;align-items:flex-start}.check input{width:auto;min-height:20px;margin:0}summary{cursor:pointer;padding:10px 0;font-size:14px}details+details{border-top:1px solid #e2e8e3}
.tabla{overflow:auto}table{width:100%;border-collapse:collapse;font-size:12px}th,td{padding:7px;text-align:right;border-bottom:1px solid #e2e8e3;white-space:nowrap}
@media(min-width:650px) and (min-height:720px){.app{max-height:780px;padding:20px;gap:12px}.cruceta{grid-template-columns:repeat(3,64px);grid-template-rows:repeat(3,54px)}section{padding:16px}}
@media(max-height:600px){.app{padding:7px;gap:5px}.manual{min-height:224px;padding:8px}.cruceta{grid-template-columns:repeat(3,44px);grid-template-rows:repeat(3,40px);gap:2px}.cruceta button{min-height:40px}#solarPanel{padding:8px;gap:5px}#estado{max-height:36px}footer{display:none}}
</style>
</head>

<body>
<main class="app">
<header><h1 id="tituloPantalla">Heliostato</h1><span class="intervalo">Control · Solar</span></header>
<p id="subtituloPantalla" hidden></p><p id="ayudaControl" hidden></p>
<div class="acciones">
<button id="origen" data-orden disabled onclick="referenciar()">Registrar cero</button>
<button id="habilitar" class="verde" data-orden disabled onclick="orden('/habilitar')">Habilitar</button>
<button class="rojo" onclick="orden('/seguro',true)">Detener</button>
</div>
<p id="estado" role="status" aria-live="polite">Conectando…</p>
<p id="error" role="status" aria-live="polite"></p>
<section class="manual">
<div class="cabecera"><h2>Movimiento manual</h2><button id="homeAmbos" data-orden disabled onclick="orden('/home-ambos')">Volver a cero</button></div>
<div class="manualContenido"><div id="ejes"></div>
<div class="angulos">
<label>Motor 1 · °<input id="grado1" type="number" min="-30" max="30" step="0.1" value="0"></label>
<label>Motor 2 · °<input id="grado2" type="number" min="-30" max="30" step="0.1" value="0"></label>
<button id="posicionar" data-orden disabled onclick="enviarPosicionAngular()">Ir</button>
</div>
</div>
</section>
<section id="solarPanel">
<div class="cabecera"><h2>Seguimiento solar</h2><button onclick="document.getElementById('configSolar').showModal()">Configurar</button></div>
<p id="periodoVisible" class="intervalo">60 min · Ajustes cada 5 min</p>
<div class="solarAcciones"><button onclick="previsualizarSolar()">Ver recorrido</button><button id="iniciarSolar" class="primario" data-orden disabled onclick="activarSolar()">Activar</button></div>
<p id="resumenSolar" role="status" aria-live="polite">Configura el receptor antes de activar.</p>
<dialog id="configSolar">
<div class="dialogCabecera"><h2>Configurar seguimiento</h2><button onclick="document.getElementById('configSolar').close()">Listo</button></div>
<div class="campos">
<label>Inicio<select id="tipoInicio"><option value="ahora">Al activar</option><option value="fecha">Fecha y hora</option></select></label>
<label>Duración · min<input id="duracionSolar" type="number" min="5" max="180" step="5" value="60"></label>
</div>
<label id="campoFecha" hidden>Fecha y hora del dispositivo<input id="fechaSolar" type="datetime-local" step="60"></label>
<p id="zonaSolar"></p>
<details open><summary>Receptor y orientación</summary>
<div class="campos">
<label>Frente · azimut °<input id="rumboSolar" type="number" min="0" max="359.99" step="any" value="270"></label>
<label>Receptor · azimut °<input id="azObjetivo" type="number" min="0" max="359.99" step="any" value="270"></label>
<label>Distancia horizontal · m<input id="distObjetivo" type="number" min="0.1" max="10000" step="any" value="6"></label>
<label>Diferencia de altura · m<input id="alturaObjetivo" type="number" min="-1000" max="1000" step="any" placeholder="Por medir" required></label>
</div>
<p>Altura del receptor menos altura del espejo. Azimut verdadero: N 0°, E 90°, S 180°, O 270°. Frente: hacia quien toma la foto.</p>
</details>
<details><summary>Ubicación</summary><div class="campos">
<label>Latitud<input id="latSolar" type="number" min="-89" max="89" step="any" value="-39.82"></label>
<label>Longitud<input id="lonSolar" type="number" min="-180" max="180" step="any" value="-73.24"></label>
</div><p>Valdivia aproximada; ajusta al lugar de la prueba.</p></details>
<details><summary>Ejes y calibración</summary><div class="campos">
<label>Motor 1 positivo<select id="signo1"><option value="1">Baja derecha</option><option value="-1">Baja izquierda</option></select></label>
<label>Motor 2 positivo<select id="signo2"><option value="1">Baja trasera</option><option value="-1">Baja delantera</option></select></label>
</div><p>Visto desde la foto frontal. Modelo de pendientes independientes; verifica la inclinación combinada. Sentido opuesto estimado por simetría.</p>
<label class="check"><input id="calibracionSolar" type="checkbox">Comprobé el origen, los sentidos y las inclinaciones combinadas.</label>
</details>
</dialog>
<dialog id="recorridoSolar"><div class="dialogCabecera"><h2>Recorrido previsto</h2><button onclick="document.getElementById('recorridoSolar').close()">Cerrar</button></div>
<p>Horario de Valdivia · normal = perpendicular al espejo · valores teóricos.</p>
<div class="tabla"><table id="tablaSolar" hidden><thead><tr><th>Hora</th><th>Sol az.</th><th>Sol elev.</th><th>Normal az.</th><th>Normal elev.</th><th>M1</th><th>M2</th><th>Estado</th></tr></thead><tbody id="filasSolar"></tbody></table></div>
</dialog>
</section>
<footer>Mantén esta pestaña abierta y la pantalla encendida.</footer>
</main>
<script>
const sesionPagina = Date.now().toString(36) + Math.random().toString(36).slice(2);
let pulsacion = null;
let liberando = false;
let ocupado = false;
let conectado = false;
let estado = null;
let revision = 0;

const nombres = ['Motor 1', 'Motor 2'];

document.getElementById('ejes').innerHTML = `
<div class="cruceta">
<button class="arriba" data-mover disabled onpointerdown="comenzarJog(event,1,1)" onpointerup="soltarJog()" onpointercancel="soltarJog()" onlostpointercapture="soltarJog()" aria-label="Motor 2 positivo: baja el borde trasero">↑</button>
<button class="izquierda" data-mover disabled onpointerdown="comenzarJog(event,0,-1)" onpointerup="soltarJog()" onpointercancel="soltarJog()" onlostpointercapture="soltarJog()" aria-label="Motor 1 negativo: baja el borde izquierdo">←</button>
<span class="centro">·</span>
<button class="derecha" data-mover disabled onpointerdown="comenzarJog(event,0,1)" onpointerup="soltarJog()" onpointercancel="soltarJog()" onlostpointercapture="soltarJog()" aria-label="Motor 1 positivo: baja el borde derecho">→</button>
<button class="abajo" data-mover disabled onpointerdown="comenzarJog(event,1,-1)" onpointerup="soltarJog()" onpointercancel="soltarJog()" onlostpointercapture="soltarJog()" aria-label="Motor 2 negativo: baja el borde delantero">↓</button>
</div><div class="posiciones"><span id="dato0">M1: —</span><span id="dato1">M2: —</span></div>`;

function pintarBotones() {
  document.querySelectorAll('[data-mover]').forEach(b => { b.disabled = true; });
  document.querySelectorAll('[data-orden]').forEach(b => {
    b.disabled = true;
  });

  if (!conectado || !estado || ocupado) {
    return;
  }

  const e = estado;
  const detenido = !e.moviendo && !e.solar;
  document.getElementById("iniciarSolar").disabled = !(e.origen && !e.seguro && detenido);
  document.getElementById('ayudaControl').textContent = !e.origen
    ? 'Para mover: primero establece el origen con el espejo horizontal.'
    : e.seguro ? 'Origen registrado. Pulsa Habilitar movimiento.'
    : e.moviendo ? 'Movimiento en curso. Puedes detenerlo con Detener.'
    : 'Listo: usa las flechas o introduce ambos angulos y pulsa Ir a la posicion.';
  document.getElementById('tituloPantalla').textContent = 'Heliostato';
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
  url += (url.includes("?") ? "&" : "?") + "sesion=" + encodeURIComponent(sesionPagina);
  const controlador = new AbortController();

  const temporizador = setTimeout(() => {
    controlador.abort();
  }, 5000); // Espera HTTP: 5 s; independiente de la parada a los 15 s.

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
    if (e.solar) texto += ` Ajustes iniciados: ${e.solarIndice}/${e.solarTotal}.`;


    document.getElementById('estado').textContent = texto;

    e.ejes.forEach((x, eje) => {
      const minimo = x.min === null ? 'pendiente' : x.min;
      const maximo = x.max === null ? 'pendiente' : x.max;

      document.getElementById('dato' + eje).textContent =
        `M${eje+1}: ${x.pos} pasos`;
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
  if (!conectado || (estado && estado.solar) || event.button !== 0 || pulsacion || liberando || ocupado || !estado || estado.moviendo || estado.seguro || !estado.origen) return;
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

// Astronomía aproximada USNO, coordenadas ENU (este, norte, arriba).
// https://aa.usno.navy.mil/faq/sun_approx
// No requiere red: Date aporta UTC y el ESP32 ejecuta el plan con millis().
const rad = Math.PI / 180;
const envolver = x => ((x % 360) + 360) % 360;
const producto = (a,b) => a.reduce((s,x,i) => s+x*b[i],0);
function unitario(v) {
  const m = Math.hypot(...v);
  if (!Number.isFinite(m) || m < 1e-9) throw Error('Geometría degenerada: Sol y receptor en direcciones opuestas.');
  return v.map(x=>x/m);
}
function calcularSol(epoch, lat, lon) {
  const jd = epoch / 86400000 + 2440587.5;
  const d = jd - 2451545;
  const g = envolver(357.529 + 0.98560028*d)*rad;
  const q = envolver(280.459 + 0.98564736*d);
  const l = (q + 1.915*Math.sin(g) + 0.020*Math.sin(2*g))*rad;
  const e = (23.439 - 0.00000036*d)*rad;
  const ra = Math.atan2(Math.cos(e)*Math.sin(l),Math.cos(l));
  const dec = Math.asin(Math.sin(e)*Math.sin(l));
  // Tiempo solar aparente a partir de EqT = q/15 - RA (horas).
  const eqMin = (((q - ra/rad + 180) % 360 + 360) % 360 - 180)*4;
  const fecha = new Date(epoch);
  const utcMin = fecha.getUTCHours()*60 + fecha.getUTCMinutes() + fecha.getUTCSeconds()/60 + fecha.getUTCMilliseconds()/60000;
  const h = (utcMin/4 + lon + eqMin/4 - 180)*rad;
  const phi = lat*rad;
  const v = [-Math.cos(dec)*Math.sin(h),
    Math.cos(phi)*Math.sin(dec)-Math.sin(phi)*Math.cos(dec)*Math.cos(h),
    Math.sin(phi)*Math.sin(dec)+Math.cos(phi)*Math.cos(dec)*Math.cos(h)];
  return {v,az:envolver(Math.atan2(v[0],v[1])/rad),el:Math.asin(Math.max(-1,Math.min(1,v[2])))/rad};
}
function orientarEspejo(sol, cfg) {
  const a=cfg.az*rad;
  const objetivo=unitario([cfg.dist*Math.sin(a),cfg.dist*Math.cos(a),cfg.altura]);
  // Normal = bisectriz de la dirección hacia el Sol y hacia el receptor.
  const n=unitario(sol.v.map((x,i)=>x+objetivo[i]));
  if (n[2] <= 0) throw Error('La orientación requiere la cara inferior del espejo.');
  const rumbo=cfg.rumbo*rad;
  // Derecha vista en la foto frontal: el observador mira hacia la estructura.
  // El frente apunta desde la estructura hacia el observador/receptor.
  const derecha=[-Math.cos(rumbo),Math.sin(rumbo),0];
  const frente=[Math.sin(rumbo),Math.cos(rumbo),0];
  // +M1: derecha abajo. +M2: trasera abajo (normal hacia atrás).
  const g1=cfg.signo1*Math.atan2(producto(n,derecha),n[2])/rad;
  const g2=-cfg.signo2*Math.atan2(producto(n,frente),n[2])/rad;
  return {g1,g2,n,objetivo,azNormal:envolver(Math.atan2(n[0],n[1])/rad),elNormal:Math.asin(n[2])/rad};
}
function construirPlan(cfg, inicio) {
  if (![inicio,cfg.lat,cfg.lon,cfg.rumbo,cfg.az,cfg.dist,cfg.altura,cfg.minutos,cfg.signo1,cfg.signo2].every(Number.isFinite) ||
      cfg.lat < -89 || cfg.lat > 89 || Math.abs(cfg.lon)>180 || cfg.rumbo<0 || cfg.rumbo>=360 || cfg.az<0 || cfg.az>=360 ||
      cfg.dist<=0 || cfg.minutos<5 || cfg.minutos>180 || cfg.minutos%5 || ![1,-1].includes(cfg.signo1) || ![1,-1].includes(cfg.signo2)) throw Error('Revisa los datos del lugar, receptor y duración.');
  const year=new Date(inicio).getUTCFullYear();
  if (year<2000 || year>2100) throw Error('La fecha debe estar entre 2000 y 2100.');
  const puntos=[];
  for (let minuto=0; minuto<=cfg.minutos; minuto+=5) {
    const epoch=inicio+minuto*60000;
    const sol=calcularSol(epoch,cfg.lat,cfg.lon);
    let punto={epoch,az:sol.az,el:sol.el,g1:null,g2:null,error:''};
    try {
      if (sol.el<5) throw Error('Sol bajo (menos de 5°)');
      Object.assign(punto,orientarEspejo(sol,cfg));
      if (Math.abs(punto.g1)>30 || Math.abs(punto.g2)>30) throw Error('Fuera de ±30°');
    } catch(e) {punto.error=e.message;}
    puntos.push(punto);
  }
  return puntos;
}
function leerConfiguracionSolar() {
  function valor(id) {
    const e=document.getElementById(id);
    if (!e.value.trim() || !e.checkValidity()) throw Error('Completa o corrige: '+e.parentElement.textContent.trim());
    return Number(e.value);
  }
  return {lat:valor('latSolar'),lon:valor('lonSolar'),rumbo:valor('rumboSolar'),az:valor('azObjetivo'),dist:valor('distObjetivo'),altura:valor('alturaObjetivo'),minutos:valor('duracionSolar'),signo1:valor('signo1'),signo2:valor('signo2')};
}
function horaSolar(epoch) {
  return new Intl.DateTimeFormat('es-CL',{timeZone:'America/Santiago',hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false}).format(new Date(epoch));
}
function prepararPlanSolar() {
  const cfg=leerConfiguracionSolar();
  const ahora=Date.now();
  const programado=document.getElementById('tipoInicio').value==='fecha';
  const fecha=document.getElementById('fechaSolar').value;
  if (programado && !fecha) throw Error('Selecciona fecha y hora de inicio.');
  // El margen permite enviar el plan; la primera posición se ordena a esa hora,
  // no se promete haber llegado a ella antes de que empiece el movimiento.
  const inicio=programado ? new Date(fecha).getTime() : ahora+10000;
  if (!Number.isFinite(inicio) || inicio<ahora+5000 || inicio>ahora+86400000) throw Error('Programa el inicio entre 5 segundos y 24 horas desde ahora.');
  return {cfg,inicio,puntos:construirPlan(cfg,inicio)};
}
function mostrarPlanSolar(plan) {
  const tbody=document.getElementById('filasSolar'); tbody.replaceChildren();
  for (const p of plan.puntos) {
    const tr=document.createElement('tr');
    [horaSolar(p.epoch),p.az.toFixed(2)+'°',p.el.toFixed(2)+'°',p.azNormal===undefined?'—':p.azNormal.toFixed(2)+'°',p.elNormal===undefined?'—':p.elNormal.toFixed(2)+'°',p.g1===null?'—':p.g1.toFixed(2)+'°',p.g2===null?'—':p.g2.toFixed(2)+'°',p.error||'OK'].forEach(t=>{
      const td=document.createElement('td');td.textContent=t;tr.appendChild(td);
    });
    tbody.appendChild(tr);
  }
  document.getElementById('tablaSolar').hidden=false;
  const invalidos=plan.puntos.filter(p=>p.error).length;
  document.getElementById('resumenSolar').textContent=invalidos
    ? 'No se puede ejecutar: '+invalidos+' posiciones no válidas. Cambia el horario o la geometría; no se recortan ángulos.'
    : plan.puntos.length+' posiciones · '+horaSolar(plan.inicio)+'–'+horaSolar(plan.puntos.at(-1).epoch)+' · Valdivia';
  return !invalidos;
}
function previsualizarSolar() {
  try {mostrarPlanSolar(prepararPlanSolar());document.getElementById('recorridoSolar').showModal();}
  catch(e) {document.getElementById('resumenSolar').textContent=e.message;document.getElementById('tablaSolar').hidden=true;}
}
async function activarSolar() {
  if (ocupado || pulsacion || liberando || !conectado || !estado || estado.seguro || estado.moviendo || estado.solar || !estado.origen) return;
  try {
    const plan=prepararPlanSolar();
    if (!mostrarPlanSolar(plan)) {document.getElementById('recorridoSolar').showModal();return;}
    if (!document.getElementById('calibracionSolar').checked) throw Error('Primero comprueba y confirma la calibración de ambos ejes en Ejes y calibración.');
    const cuerpo=new URLSearchParams({n:String(plan.puntos.length),delay:String(Math.round(plan.inicio-Date.now()))});
    plan.puntos.forEach((p,i)=>{cuerpo.set('p'+i+'_0',p.g1.toFixed(6));cuerpo.set('p'+i+'_1',p.g2.toFixed(6));});
    revision++; ocupado=true; const miRevision=revision; pintarBotones();
    try {
      const r=await peticion('/solar',{method:'POST',body:cuerpo});
      const texto=await r.text();document.getElementById('error').textContent=texto;
      if (!r.ok) throw Error(texto);
    } catch(e) {
      conectado=false;throw Error(e.message+' Consulta el estado antes de repetir la activación.');
    } finally {
      if (miRevision===revision) {ocupado=false;estado=null;pintarBotones();}
    }
  } catch(e) {document.getElementById('resumenSolar').textContent=e.message;}
}
document.getElementById('tipoInicio').addEventListener('change',()=>{
  document.getElementById('campoFecha').hidden=document.getElementById('tipoInicio').value!=='fecha';
});
document.getElementById('zonaSolar').textContent='La fecha ingresada usa la zona del dispositivo: '+Intl.DateTimeFormat().resolvedOptions().timeZone+'. La tabla se muestra en America/Santiago. Verifica el reloj antes de activar.';
document.getElementById('solarPanel').addEventListener('input',()=>{
  document.getElementById('periodoVisible').textContent=document.getElementById('duracionSolar').value+' min · Ajustes cada 5 min';
  document.getElementById('tablaSolar').hidden=true;
  document.getElementById('resumenSolar').textContent='Datos modificados. Revisa nuevamente el recorrido antes de activar.';
});

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
  solarActivo = false;
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
  if (moviendo || segundoPendiente || (solarActivo && !respuestaInterna)) {
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

  if (!respuestaInterna) {
    clienteControl = server.client().remoteIP(); sesionControl = server.arg("sesion");
    hayClienteControl = true;
  }
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
      jogActivo = false;
      tokenJog = "";

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
  if (hayClienteControl && server.client().remoteIP() == clienteControl && server.arg("sesion") == sesionControl) ultimoContactoMs = millis();

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

  json += ",\"solar\":";
  json += solarActivo ? "true" : "false";
  json += ",\"solarIndice\":";
  json += String(indiceSolar);
  json += ",\"solarTotal\":";
  json += String(puntosSolar);
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

  server.on("/solar", HTTP_POST, recibirPlanSolar);

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
    clienteControl = server.client().remoteIP(); sesionControl = server.arg("sesion");
    hayClienteControl = true;
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
  actualizarSolar();
  // Completar el pulso HIGH antes de atender HTTP.
  if (!pulsoAlto) {
    server.handleClient();
  }
  actualizarMovimiento();
}


// El navegador entrega un plan calculado y visible; el firmware vuelve a
// validar todos los puntos antes de modificar el plan activo o emitir pulsos.
void recibirPlanSolar() {
  if (!comprobarDetenido()) return;
  if (!origenEstablecido || modoSeguro || !rangoConfirmado) {
    responder(403, "Establece origen y habilita antes de programar."); return;
  }
  double cantidad, demora;
  auto numero = [](const String& nombre, double& v) -> bool {
    String s = server.arg(nombre); s.trim();
    if (!s.length() || s.length() > 24) return false;
    char* fin; errno = 0; v = strtod(s.c_str(), &fin);
    return fin != s.c_str() && *fin == '\0' && errno != ERANGE && isfinite(v);
  };
  if (!numero("n", cantidad) || cantidad != floor(cantidad) || cantidad < 2 || cantidad > maxPuntosSolar ||
      !numero("delay", demora) || demora < 0 || demora > 86400000 || demora != floor(demora)) {
    responder(400, "Periodo o inicio invalido (maximo 24 horas de espera)."); return;
  }
  long temporal[maxPuntosSolar][2];
  for (int i = 0; i < int(cantidad); ++i) {
    for (int eje = 0; eje < 2; ++eje) {
      double g;
      String nombre = String("p") + String(i) + "_" + String(eje);
      if (!numero(nombre, g) || fabs(g) > 30) {
        responder(400, "Un punto excede +/-30 grados o no es numerico."); return;
      }
      temporal[i][eje] = gradosAPasos(g);
      if (temporal[i][eje] < limiteMin[eje] || temporal[i][eje] > limiteMax[eje] ||
          labs(temporal[i][eje]) > recorridoMedido) {
        responder(400, "Un punto supera el recorrido permitido."); return;
      }
    }
    long previo0 = i ? temporal[i-1][0] : posicion[0];
    long previo1 = i ? temporal[i-1][1] : posicion[1];
    uint32_t pasos = labs(temporal[i][0]-previo0) + labs(temporal[i][1]-previo1);
    if (pasos * (intervaloFijoUs / 1000) + 10000 >= periodoSolarMs) {
      responder(400, "Un movimiento no cabe en el intervalo de cinco minutos."); return;
    }
  }
  puntosSolar = int(cantidad); indiceSolar = 0;
  for (int i=0; i<puntosSolar; ++i) for (int e=0; e<2; ++e) planSolar[i][e]=temporal[i][e];
  demoraSolarMs = uint32_t(demora); inicioSolarMs = millis();
  clienteControl = server.client().remoteIP(); sesionControl = server.arg("sesion"); hayClienteControl = true;
  ultimoContactoMs = millis(); solarActivo = true;
  mensaje = "Seguimiento programado. Manten esta pagina abierta.";
  responder(200, mensaje);
}

void actualizarSolar() {
  if (!solarActivo || modoSeguro) return;
  uint32_t transcurrido = uint32_t(millis() - inicioSolarMs);
  if (indiceSolar == puntosSolar) {
    if (!moviendo && !segundoPendiente) {
      solarActivo = false;
      mensaje = "Prueba terminada. Se conserva la ultima posicion con torque.";
    }
    return;
  }
  uint32_t vencimiento = demoraSolarMs + uint32_t(indiceSolar) * periodoSolarMs;
  if (transcurrido < vencimiento) return;
  if (moviendo || segundoPendiente || transcurrido - vencimiento > 2000) {
    activarModoSeguro("Seguimiento detenido: ajuste atrasado o movimiento sin terminar."); return;
  }
  if (planSolar[indiceSolar][0] == posicion[0] && planSolar[indiceSolar][1] == posicion[1]) {
    ++indiceSolar; mensaje = "Seguimiento: posicion conservada."; return;
  }
  respuestaInterna = true;
  bool ok = iniciarCombinado(planSolar[indiceSolar][0], planSolar[indiceSolar][1]);
  respuestaInterna = false;
  if (!ok) { activarModoSeguro("No se pudo iniciar el ajuste solar."); return; }
  ++indiceSolar;
}

