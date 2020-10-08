#define MARGEM_DE_SEGURANCA_LIFTOFF 7500
#define MARGEM_DE_SEGURANCA_APOGEU 10
#define MARGEM_DE_SEGURANCA_POUSO 20

int16_t valor_aceleracao_inicial = 0;
int16_t *ac_eixo_principal;
int16_t ultima_aceleracao_eixo_principal;

float primeira_altura, altura, ultima_altura, altura_maxima = 0.0f;


int16_t* descobreEixoPrincipal(){
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

bool verificaBurnOut(){
	if(*ac_eixo_principal < (ultima_aceleracao_eixo_principal + MARGEM_DE_SEGURANCA_LIFTOFF)) return true;
	ultima_aceleracao_eixo_principal = *ac_eixo_principal;
	return false;
}

void AtualizaAlturaMaxima(){
	if(altura >= ultima_altura - MARGEM_DE_SEGURANCA_APOGEU) altura_maxima = altura-primeira_altura;
}