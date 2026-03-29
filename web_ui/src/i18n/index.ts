import i18n from 'i18next';
import { initReactI18next } from 'react-i18next';
import { zhCommon } from './locales/zh-CN/common';
import { zhControl } from './locales/zh-CN/control';
import { enCommon } from './locales/en/common';
import { enControl } from './locales/en/control';

void i18n.use(initReactI18next).init({
  resources: {
    'zh-CN': {
      common: zhCommon,
      control: zhControl,
    },
    en: {
      common: enCommon,
      control: enControl,
    },
  },
  lng: 'zh-CN',
  fallbackLng: 'en',
  defaultNS: 'common',
  ns: ['common', 'control'],
  interpolation: {
    escapeValue: false,
  },
});

export default i18n;
