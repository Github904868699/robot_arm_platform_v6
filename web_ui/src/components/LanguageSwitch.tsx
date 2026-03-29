import { useTranslation } from 'react-i18next';

export function LanguageSwitch() {
  const { i18n, t } = useTranslation('common');

  return (
    <div className="lang-switch" role="group" aria-label={t('language')}>
      <button
        className={i18n.language === 'zh-CN' ? 'active' : ''}
        onClick={() => void i18n.changeLanguage('zh-CN')}
      >
        {t('chinese')}
      </button>
      <button
        className={i18n.language === 'en' ? 'active' : ''}
        onClick={() => void i18n.changeLanguage('en')}
      >
        {t('english')}
      </button>
    </div>
  );
}
