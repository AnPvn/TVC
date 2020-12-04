#define MARGEM_DE_SEGURANCA_LIFTOFF 7500
#define MARGEM_DE_SEGURANCA_APOGEU 10
#define MARGEM_DE_SEGURANCA_POUSO 20

int16_t ax, ay, az; // mpu
int16_t gx, gy, gz; // mpu
const float lsb = 16384.00; // mpu

int16_t valor_aceleracao_inicial = 0;
int16_t *ac_eixo_principal;
int16_t ultima_aceleracao_eixo_principal;

float primeira_altura, altura, ultima_altura, altura_maxima = 0.0f;


void descobreEixoPrincipal(){
  updateMPU();

  ac_eixo_principal = (ax/lsb >= 0.9) ? &ax : ((ay/lsb >= 0.9) ? &ay : ((az/lsb >= 0.9) ? &az : &ax));

  valor_aceleracao_inicial = *ac_eixo_principal;
}

bool verificaLancamentoIniciado(){
  if(*ac_eixo_principal > valor_aceleracao_inicial + MARGEM_DE_SEGURANCA_LIFTOFF ) return true;
    return false;
}

bool verificaApogeu(){
  if(getAltitude() < ultima_altura - MARGEM_DE_SEGURANCA_APOGEU) return true;
  return false;
}

bool verificaLancamentoTerminado(){
  if((altura - primeira_altura) <= MARGEM_DE_SEGURANCA_POUSO){
    delay(5000);
    return true;
  }
  return false;
}

bool verificaBurnout(){
  if(*ac_eixo_principal < (ultima_aceleracao_eixo_principal + MARGEM_DE_SEGURANCA_LIFTOFF)) return true;
  ultima_aceleracao_eixo_principal = *ac_eixo_principal;
  return false;
}

void AtualizaAlturaMaxima(){
  if(altura >= ultima_altura - MARGEM_DE_SEGURANCA_APOGEU) altura_maxima = altura-primeira_altura;
}

void setPrimeiraAltura(float valor){ primeira_altura = valor; }
void setUltimaAltura(float valor){ ultima_altura = valor; }
void setAltura(float valor){ altura = valor; }

float getAltura(){ return altura; }
float getAlturaMaxima() { return altura_maxima; }

float getAx(){ updateMPU(); return ax; }
float getAy(){ updateMPU(); return ay; }
float getAz(){ updateMPU(); return az; }
float getGx(){ updateMPU(); return gx; }
float getGy(){ updateMPU(); return gy; }
float getGz(){ updateMPU(); return gz; }

void setAx(int16_t AX){ ax = AX; }
void setAy(int16_t AY){ ay = AY; }
void setAz(int16_t AZ){ az = AZ; }
void setGx(int16_t GX){ gx = GX; }
void setGy(int16_t GY){ gy = GY; }
void setGz(int16_t GZ){ gz = GZ; }

int16_t getAcEixoPrincipal(){ return *ac_eixo_principal; }
