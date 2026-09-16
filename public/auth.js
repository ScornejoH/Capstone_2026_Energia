/*
 * auth.js — acceso de solo lectura para el público y control para el equipo.
 *
 * La lectura de telemetría y parámetros es pública (ver database.rules.json).
 * Para escribir (params) hay que iniciar sesión con correo/contraseña con una
 * cuenta registrada en /usuarios/<uid> = true (se agrega desde la consola de
 * Firebase). Este módulo agrega un widget flotante de sesión y expone
 * canWrite() para que cada página bloquee los comandos en modo lectura.
 */
import { getAuth, onAuthStateChanged, signInWithEmailAndPassword, signOut }
  from "https://www.gstatic.com/firebasejs/10.12.2/firebase-auth.js";
import { ref, get }
  from "https://www.gstatic.com/firebasejs/10.12.2/firebase-database.js";

const CSS = `
#auth-widget{position:fixed;right:16px;bottom:16px;z-index:9999;font:13px/1.4 system-ui,sans-serif;color:#eef4fb}
#auth-widget .aw-pill{display:flex;align-items:center;gap:8px;background:#16202e;border:1px solid rgba(143,163,189,.3);
  border-radius:999px;padding:6px 8px 6px 12px;box-shadow:0 4px 16px rgba(0,0,0,.35)}
#auth-widget .aw-dot{width:8px;height:8px;border-radius:50%;background:#8fa3bd}
#auth-widget[data-state="editor"] .aw-dot{background:#16b399}
#auth-widget[data-state="noperm"] .aw-dot{background:#f0a92b}
#auth-widget button{font:inherit;color:#eef4fb;background:#1c2a3a;border:1px solid rgba(143,163,189,.3);
  border-radius:999px;padding:4px 10px;cursor:pointer}
#auth-widget button:hover{border-color:#3d8ce0}
#auth-widget form{margin-top:8px;display:flex;flex-direction:column;gap:6px;background:#16202e;
  border:1px solid rgba(143,163,189,.3);border-radius:10px;padding:10px;width:230px}
#auth-widget form[hidden]{display:none}
#auth-widget input{font:inherit;color:#eef4fb;background:#0e1621;border:1px solid rgba(143,163,189,.3);border-radius:6px;padding:6px 8px}
#auth-widget .aw-err{color:#e2504a;font-size:12px;min-height:1em}
#auth-toast{position:fixed;left:50%;bottom:72px;transform:translateX(-50%);z-index:9999;background:#e2504a;color:#fff;
  font:13px system-ui,sans-serif;padding:8px 14px;border-radius:8px;opacity:0;transition:opacity .2s;pointer-events:none}
#auth-toast.show{opacity:1}
`;

let editor = false;

export function canWrite() {
  if (editor) return true;
  toast("Modo solo lectura: inicia sesión con una cuenta del equipo para enviar comandos.");
  return false;
}

function toast(msg) {
  let t = document.getElementById("auth-toast");
  if (!t) { t = document.createElement("div"); t.id = "auth-toast"; document.body.appendChild(t); }
  t.textContent = msg;
  t.classList.add("show");
  clearTimeout(t._h);
  t._h = setTimeout(() => t.classList.remove("show"), 3500);
}

export function setupAuth(app, db) {
  const auth = getAuth(app);

  const style = document.createElement("style");
  style.textContent = CSS;
  document.head.appendChild(style);

  const w = document.createElement("div");
  w.id = "auth-widget";
  w.innerHTML = `
    <div class="aw-pill"><span class="aw-dot"></span><span class="aw-label">Solo lectura</span>
      <button type="button" class="aw-btn">Iniciar sesión</button></div>
    <form hidden>
      <input type="email" name="email" placeholder="Correo" autocomplete="username" required>
      <input type="password" name="password" placeholder="Contraseña" autocomplete="current-password" required>
      <button type="submit">Entrar</button>
      <div class="aw-err"></div>
    </form>`;
  document.body.appendChild(w);

  const label = w.querySelector(".aw-label");
  const btn   = w.querySelector(".aw-btn");
  const form  = w.querySelector("form");
  const err   = w.querySelector(".aw-err");

  btn.addEventListener("click", async () => {
    if (auth.currentUser) { await signOut(auth); return; }
    form.hidden = !form.hidden;
    if (!form.hidden) form.email.focus();
  });

  form.addEventListener("submit", async (e) => {
    e.preventDefault();
    err.textContent = "";
    try {
      await signInWithEmailAndPassword(auth, form.email.value.trim(), form.password.value);
      form.reset();
      form.hidden = true;
    } catch (ex) {
      err.textContent = "No se pudo iniciar sesión.";
      console.error("[Auth]", ex);
    }
  });

  onAuthStateChanged(auth, async (user) => {
    editor = false;
    if (!user) {
      w.dataset.state = "viewer";
      label.textContent = "Solo lectura";
      btn.textContent = "Iniciar sesión";
      return;
    }
    try {
      editor = (await get(ref(db, `usuarios/${user.uid}`))).val() === true;
    } catch (ex) {
      console.error("[Auth] No se pudo verificar permisos:", ex);
    }
    w.dataset.state = editor ? "editor" : "noperm";
    label.textContent = editor ? "Control habilitado" : "Cuenta sin permisos";
    btn.textContent = "Salir";
  });

  return auth;
}
