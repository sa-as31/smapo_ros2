<template>
  <div class="login-stage">
    <article class="login-hero">
      <span class="eyebrow">Operations Access</span>
      <h1>让申请、调度与执行落在同一套运营工作台里。</h1>
      <p class="login-hero-copy">
        面向申请人、调度管理员与飞手的统一入口，聚焦任务排队、审批分发、联合执行与回放归档四条主链路。
      </p>
      <div class="login-hero-grid">
        <div class="login-hero-note">
          <strong>申请排队</strong>
          <span>提交任务名称、地点与时间，立即进入审核队列。</span>
        </div>
        <div class="login-hero-note">
          <strong>调度审批</strong>
          <span>按地图、飞手与执行窗口完成分发与调度。</span>
        </div>
        <div class="login-hero-note">
          <strong>执行归档</strong>
          <span>实时状态、现场记录与历史回放统一归档。</span>
        </div>
      </div>
    </article>

    <article class="login-card editorial-card">
      <span class="eyebrow">System Access</span>
      <h2>{{ authMode === "login" ? "进入系统" : "创建账号" }}</h2>
      <p class="login-sub">{{ authMode === "login" ? "选择席位后登录当前工作台。" : "注册后会直接进入对应角色工作台。" }}</p>
      <div class="login-mode-switch" role="tablist" aria-label="认证模式">
        <button :class="{ active: authMode === 'login' }" @click="setAuthMode('login')">登录</button>
        <button :class="{ active: authMode === 'register' }" @click="setAuthMode('register')">注册</button>
      </div>
      <div class="login-role-row">
        <button class="role-toggle" :class="{ active: loginRole === 'requester' }" @click="setLoginRole('requester')">申请人</button>
        <button class="role-toggle" :class="{ active: loginRole === 'admin' }" @click="setLoginRole('admin')">管理员</button>
        <button class="role-toggle" :class="{ active: loginRole === 'executor' }" @click="setLoginRole('executor')">飞手</button>
      </div>
      <div class="login-access-strip">
        <span class="login-access-chip">{{ roleHintText }}</span>
        <span class="login-access-chip">{{ authMode === "login" ? "默认密码 1" : "密码至少 6 位" }}</span>
      </div>

      <label v-if="authMode === 'register'" class="login-label">
        姓名
        <input v-model="registerDisplayName" type="text" autocomplete="name" placeholder="请输入姓名或昵称" />
      </label>
      <label class="login-label">
        账号
        <input v-model="loginUsername" type="text" autocomplete="username" placeholder="请输入账号" />
      </label>
      <label v-if="authMode === 'register'" class="login-label">
        部门
        <input v-model="registerDepartment" type="text" autocomplete="organization" placeholder="例如：任务申请组 / 运营飞行组" />
      </label>
      <label class="login-label">
        密码
        <input
          v-model="loginPassword"
          type="password"
          :autocomplete="authMode === 'login' ? 'current-password' : 'new-password'"
          placeholder="请输入密码"
          @keyup.enter="submitAuth"
        />
      </label>
      <label v-if="authMode === 'register'" class="login-label">
        确认密码
        <input v-model="registerConfirmPassword" type="password" autocomplete="new-password" placeholder="请再次输入密码" @keyup.enter="submitAuth" />
      </label>
      <p v-if="authStore.state.authError" class="role-error">{{ authStore.state.authError }}</p>
      <div class="btn-row login-actions">
        <button class="btn btn-dark" :disabled="authStore.state.authBusy" @click="submitAuth">
          {{ actionButtonText }}
        </button>
      </div>
    </article>
  </div>
</template>

<script setup>
import { ref, computed, onMounted } from 'vue';
import { useRouter } from 'vue-router';
import { useAuthStore } from '../stores/auth';

const router = useRouter();
const authStore = useAuthStore();

const authMode = ref("login");
const loginRole = ref("requester");
const loginUsername = ref("");
const loginPassword = ref("");
const registerConfirmPassword = ref("");
const registerDisplayName = ref("");
const registerDepartment = ref("");

const roleHintText = computed(() => {
  if (loginRole.value === "admin") return "管理员示例：admin";
  if (loginRole.value === "requester") return "申请人示例：requester01";
  return "飞手示例：executor01";
});

const actionButtonText = computed(() => {
  if (authStore.state.authBusy) return authMode.value === "login" ? "登录中..." : "注册中...";
  return authMode.value === "login" ? "登录系统" : "注册并进入";
});

function prefillByRole(role) {
  if (authMode.value !== "login") return;
  const first = authStore.state.authAccounts.find((item) => item.role === role);
  if (first) loginUsername.value = first.username;
}

function setAuthMode(mode) {
  authMode.value = mode === "register" ? "register" : "login";
  loginPassword.value = "";
  registerConfirmPassword.value = "";
  if (authMode.value === "login") {
    prefillByRole(loginRole.value);
  } else if (loginUsername.value === roleHintText.value.split("：")[1]) {
    loginUsername.value = "";
  }
}

function setLoginRole(role) {
  loginRole.value = role === "admin" || role === "requester" ? role : "executor";
  prefillByRole(loginRole.value);
}

function pushRoleHome() {
  const role = authStore.state.currentUser?.role;
  if (role === 'admin') router.push('/admin');
  else if (role === 'requester') router.push('/requester');
  else router.push('/executor');
}

async function submitAuth() {
  let success = false;
  if (authMode.value === "login") {
    success = await authStore.login(loginRole.value, loginUsername.value, loginPassword.value);
  } else {
    success = await authStore.register({
      role: loginRole.value,
      username: loginUsername.value,
      password: loginPassword.value,
      confirm_password: registerConfirmPassword.value,
      display_name: registerDisplayName.value,
      department: registerDepartment.value,
    });
  }
  if (success) {
    pushRoleHome();
  }
}

onMounted(() => {
  if (authStore.state.authAccounts.length === 0) {
    authStore.loadAuthOptions().then(() => {
      prefillByRole(loginRole.value);
    });
  } else {
    prefillByRole(loginRole.value);
  }
});
</script>
