% retorna o codigo ASCII da tecla pressionada.
% casos particulares:
% PgUp (acima) = 180
% PgDown (abaixo)  = 181
% End (direita) = 182
% Home (esquerda) = 183

function key = getKey() 
k = kbhit(1);
if (toascii(k) == 27)     % ascii Esc
  k = kbhit();    % '['
  k = kbhit();    % 'A', 'B', ...
  key = toascii(k) + 115;
elseif 1
  key = toascii(k);
end
endfunction
